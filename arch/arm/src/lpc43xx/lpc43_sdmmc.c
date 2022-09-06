/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_sdmmc.c
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
#include <debug.h>
#include <errno.h>

#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/sdio.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mmcsd.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/lpc43_pinconfig.h"
#include "lpc43_cgu.h"
#include "lpc43_ccu.h"
#include "lpc43_gpio.h"
#include "lpc43_sdmmc.h"

#include <arch/board/board.h>

#ifdef CONFIG_LPC43_SDMMC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MCI_DMADES0_OWN         (1UL << 31)
#define MCI_DMADES0_CH          (1 << 4)
#define MCI_DMADES0_FS          (1 << 3)
#define MCI_DMADES0_LD          (1 << 2)
#define MCI_DMADES0_DIC         (1 << 1)
#define MCI_DMADES1_MAXTR       4096
#define MCI_DMADES1_BS1(x)      (x)

/* Configuration ************************************************************/

/* Required system configuration options in the sched/Kconfig:
 *
 *   CONFIG_SCHED_WORKQUEUE -- Callback support requires work queue support.
 *
 * Driver-specific configuration options in the drivers/mmcd Kdonfig:
 *
 *   CONFIG_SDIO_MUXBUS - Setting this configuration enables some locking
 *     APIs to manage concurrent accesses on the SD card bus.  This is not
 *     needed for the simple case of a single SD card slot, for example.
 *   CONFIG_SDIO_WIDTH_D1_ONLY - This may be selected to force the driver
 *     operate with only a single data line (the default is to use all
 *     4 SD data lines).
 *   CONFIG_MMCSD_HAVE_CARDDETECT - Select if the SD slot supports a card
 *     detect pin.
 *   CONFIG_MMCSD_HAVE_WRITEPROTECT - Select if the SD slots supports a
 *     write protected pin.
 *
 * Driver-specific configuration options in the arch/arm/src/lpc43xx/Kconfig
 *
 *   CONFIG_LPC43_SDMMC_PWRCTRL - Select if the board supports an output
 *     pin to enable power to the SD slot.
 *   CONFIG_LPC43_SDMMC_DMA - Enable SD card DMA.  This is a marginally
 *     optional.  For most usages, SD accesses will cause data overruns if
 *     used without DMA.  This will also select CONFIG_SDIO_DMA.
 *   CONFIG_LPC43_SDMMC_REGDEBUG - Enables some very low-level debug output
 *     This also requires CONFIG_DEBUG_MEMCARD_INFO
 */

#ifndef CONFIG_SCHED_WORKQUEUE
#  error "Callback support requires CONFIG_SCHED_WORKQUEUE"
#endif

#ifndef CONFIG_SDIO_BLOCKSETUP
#  error "Driver requires CONFIG_SDIO_BLOCKSETUP to be set"
#endif

/* Timing : 100mS short timeout, 2 seconds for long one */

#define SDCARD_CMDTIMEOUT       MSEC2TICK(100)
#define SDCARD_LONGTIMEOUT      MSEC2TICK(2000)

/* Type of Card Bus Size */

#define SDCARD_BUS_D1           0
#define SDCARD_BUS_D4           1
#define SDCARD_BUS_D8           0x100

/* FIFO size in bytes */

#define LPC43_TXFIFO_SIZE       (LPC43_TXFIFO_DEPTH | LPC43_TXFIFO_WIDTH)
#define LPC43_RXFIFO_SIZE       (LPC43_RXFIFO_DEPTH | LPC43_RXFIFO_WIDTH)

/* Number of DMA Descriptors */

#define NUM_DMA_DESCRIPTORS     (1 + (0x10000 / MCI_DMADES1_MAXTR))

/* Data transfer interrupt mask bits */

#define SDCARD_RECV_MASK        (SDMMC_INT_DTO  | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                                 SDMMC_INT_EBE  | SDMMC_INT_RXDR | SDMMC_INT_SBE)
#define SDCARD_SEND_MASK        (SDMMC_INT_DTO  | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                                 SDMMC_INT_EBE  | SDMMC_INT_TXDR | SDMMC_INT_SBE)

#define SDCARD_DMARECV_MASK     (SDMMC_INT_DTO  | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                                 SDMMC_INT_SBE  | SDMMC_INT_EBE)
#define SDCARD_DMASEND_MASK     (SDMMC_INT_DTO  | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                                 SDMMC_INT_EBE)

#define SDCARD_DMAERROR_MASK    (SDMMC_IDINTEN_FBE | SDMMC_IDINTEN_DU | \
                                 SDMMC_IDINTEN_AIS)

#define SDCARD_TRANSFER_ALL     (SDMMC_INT_DTO  | SDMMC_INT_DCRC | SDMMC_INT_DRTO | \
                                 SDMMC_INT_EBE  | SDMMC_INT_TXDR | SDMMC_INT_RXDR | \
                                 SDMMC_INT_SBE)

/* Event waiting interrupt mask bits */

#define SDCARD_INT_RESPERR      (SDMMC_INT_RE   | SDMMC_INT_RCRC | SDMMC_INT_RTO)

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
#  define SDCARD_INT_CDET        SDMMC_INT_CDET
#else
#  define SDCARD_INT_CDET        0
#endif

#define SDCARD_CMDDONE_STA      (SDMMC_INT_CDONE)
#define SDCARD_RESPDONE_STA     (0)

#define SDCARD_CMDDONE_MASK     (SDMMC_INT_CDONE)
#define SDCARD_RESPDONE_MASK    (SDMMC_INT_CDONE | SDCARD_INT_RESPERR)
#define SDCARD_XFRDONE_MASK     (0)  /* Handled by transfer masks */

#define SDCARD_CMDDONE_CLEAR    (SDMMC_INT_CDONE)
#define SDCARD_RESPDONE_CLEAR   (SDMMC_INT_CDONE | SDCARD_INT_RESPERR)

#define SDCARD_XFRDONE_CLEAR    (SDCARD_TRANSFER_ALL)

#define SDCARD_WAITALL_CLEAR    (SDCARD_CMDDONE_CLEAR | SDCARD_RESPDONE_CLEAR | \
                                 SDCARD_XFRDONE_CLEAR)

/* Let's wait until we have both SD card transfer complete and DMA
 * complete.
 */

#define SDCARD_XFRDONE_FLAG     (1)
#define SDCARD_DMADONE_FLAG     (2)
#define SDCARD_ALLDONE          (3)

/* Card debounce time.  Number of host clocks (SD_CLK) used by debounce
 * filter logic for card detect.  typical debounce time is 5-25 ms.
 *
 * Eg. Fsd = 44MHz, ticks = 660,000
 */

#define DEBOUNCE_TICKS          (15 * (BOARD_SDMMC_FREQUENCY / 1000))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sdmmc_dma_s
{
  volatile uint32_t des0;        /* Control and status */
  volatile uint32_t des1;        /* Buffer size(s) */
  volatile uint32_t des2;        /* Buffer address pointer 1 */
  volatile uint32_t des3;        /* Buffer address pointer 2 */
};

/* This structure defines the state of the LPC43XX SD card interface */

struct lpc43_dev_s
{
  struct sdio_dev_s  dev;             /* Standard, base SD card interface */

  /* LPC43XX-specific extensions */

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
  uint32_t           xfrmask;         /* Interrupt enables for data transfer */
#ifdef CONFIG_LPC43_SDMMC_DMA
  uint32_t           dmamask;         /* Interrupt enables for DMA transfer */
#endif
  ssize_t            remaining;       /* Number of bytes remaining in the
                                       * transfer */
  bool               wrdir;           /* True: Writing False: Reading */

  /* DMA data transfer support */

  bool               widebus;         /* Required for DMA support */
#ifdef CONFIG_LPC43_SDMMC_DMA
  bool               dmamode;         /* true: DMA mode transfer */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_LPC43_SDMMC_REGDEBUG
static uint32_t lpc43_getreg(uint32_t addr);
static void lpc43_putreg(uint32_t val, uint32_t addr);
#else
# define lpc43_getreg(addr)      getreg32(addr)
# define lpc43_putreg(val,addr)  putreg32(val,addr)
#endif

/* Low-level helpers ********************************************************/

static inline void lpc43_setclock(uint32_t clkdiv);
static inline void lpc43_sdcard_clock(bool enable);
static int  lpc43_ciu_sendcmd(uint32_t cmd, uint32_t arg);
static void lpc43_enable_ints(struct lpc43_dev_s *priv);
static void lpc43_disable_allints(struct lpc43_dev_s *priv);
static void lpc43_config_waitints(struct lpc43_dev_s *priv,
              uint32_t waitmask, sdio_eventset_t waitevents,
              sdio_eventset_t wkupevents);
static void lpc43_config_xfrints(struct lpc43_dev_s *priv, uint32_t xfrmask);
#ifdef CONFIG_LPC43_SDMMC_DMA
static void lpc43_config_dmaints(struct lpc43_dev_s *priv, uint32_t xfrmask,
              uint32_t dmamask);
#endif

/* Data Transfer Helpers ****************************************************/

static void lpc43_eventtimeout(wdparm_t arg);
static void lpc43_endwait(struct lpc43_dev_s *priv,
              sdio_eventset_t wkupevent);
static void lpc43_endtransfer(struct lpc43_dev_s *priv,
              sdio_eventset_t wkupevent);

/* Interrupt Handling *******************************************************/

static int  lpc43_sdmmc_interrupt(int irq, void *context, void *arg);

/* SD Card Interface Methods ************************************************/

/* Mutual exclusion */

#ifdef CONFIG_SDIO_MUXBUS
static int  lpc43_lock(struct sdio_dev_s *dev, bool lock);
#endif

/* Initialization/setup */

static void lpc43_reset(struct sdio_dev_s *dev);
static sdio_capset_t lpc43_capabilities(struct sdio_dev_s *dev);
static uint8_t lpc43_status(struct sdio_dev_s *dev);
static void lpc43_widebus(struct sdio_dev_s *dev, bool enable);
static void lpc43_clock(struct sdio_dev_s *dev,
              enum sdio_clock_e rate);
static int  lpc43_attach(struct sdio_dev_s *dev);

/* Command/Status/Data Transfer */

static int  lpc43_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t arg);
#ifdef CONFIG_SDIO_BLOCKSETUP
static void lpc43_blocksetup(struct sdio_dev_s *dev,
              unsigned int blocklen, unsigned int nblocks);
#endif
static int  lpc43_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
              size_t nbytes);
static int  lpc43_sendsetup(struct sdio_dev_s *dev,
              const uint8_t *buffer, uint32_t nbytes);
static int  lpc43_cancel(struct sdio_dev_s *dev);

static int  lpc43_waitresponse(struct sdio_dev_s *dev, uint32_t cmd);
static int  lpc43_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  lpc43_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t rlong[4]);
static int  lpc43_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rshort);
static int  lpc43_recvnotimpl(struct sdio_dev_s *dev, uint32_t cmd,
              uint32_t *rnotimpl);

/* EVENT handler */

static void lpc43_waitenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset, uint32_t timeout);
static sdio_eventset_t lpc43_eventwait(struct sdio_dev_s *dev);
static void lpc43_callbackenable(struct sdio_dev_s *dev,
              sdio_eventset_t eventset);
static void lpc43_callback(struct lpc43_dev_s *priv);
static int  lpc43_registercallback(struct sdio_dev_s *dev,
              worker_t callback, void *arg);

#ifdef CONFIG_LPC43_SDMMC_DMA
/* DMA */

static int  lpc43_dmarecvsetup(struct sdio_dev_s *dev,
              uint8_t *buffer, size_t buflen);
static int  lpc43_dmasendsetup(struct sdio_dev_s *dev,
              const uint8_t *buffer, size_t buflen);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct lpc43_dev_s g_scard_dev =
{
  .dev =
  {
#ifdef CONFIG_SDIO_MUXBUS
    .lock             = lpc43_lock,
#endif
    .reset            = lpc43_reset,
    .capabilities     = lpc43_capabilities,
    .status           = lpc43_status,
    .widebus          = lpc43_widebus,
    .clock            = lpc43_clock,
    .attach           = lpc43_attach,
    .sendcmd          = lpc43_sendcmd,
#ifdef CONFIG_SDIO_BLOCKSETUP
    .blocksetup       = lpc43_blocksetup,
#endif
    .recvsetup        = lpc43_recvsetup,
    .sendsetup        = lpc43_sendsetup,
    .cancel           = lpc43_cancel,
    .waitresponse     = lpc43_waitresponse,
    .recv_r1          = lpc43_recvshortcrc,
    .recv_r2          = lpc43_recvlong,
    .recv_r3          = lpc43_recvshort,
    .recv_r4          = lpc43_recvnotimpl,
    .recv_r5          = lpc43_recvnotimpl,
    .recv_r6          = lpc43_recvshortcrc,
    .recv_r7          = lpc43_recvshort,
    .waitenable       = lpc43_waitenable,
    .eventwait        = lpc43_eventwait,
    .callbackenable   = lpc43_callbackenable,
    .registercallback = lpc43_registercallback,
#ifdef CONFIG_LPC43_SDMMC_DMA
    .dmarecvsetup     = lpc43_dmarecvsetup,
    .dmasendsetup     = lpc43_dmasendsetup,
#endif
  },
};

#ifdef CONFIG_LPC43_SDMMC_DMA
static struct sdmmc_dma_s g_sdmmc_dmadd[NUM_DMA_DESCRIPTORS];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_getreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   addr - The register address to read
 *
 * Returned Value:
 *   The value read from the register
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_SDMMC_REGDEBUG
static uint32_t lpc43_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval   = 0;
  static uint32_t count    = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
          if (count == 4)
            {
              mcinfo("...\n");
            }

          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
      /* Did we print "..." for the previous value? */

      if (count > 3)
        {
          /* Yes.. then show how many times the value repeated */

          mcinfo("[repeats %d more times]\n", count - 3);
        }

      /* Save the new address, value, and count */

      prevaddr = addr;
      preval   = val;
      count    = 1;
    }

  /* Show the register value read */

  mcinfo("%08x->%08x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: lpc43_putreg
 *
 * Description:
 *   This function may to used to intercept an monitor all register accesses.
 *   Clearly this is nothing you would want to do unless you are debugging
 *   this driver.
 *
 * Input Parameters:
 *   val - The value to write to the register
 *   addr - The register address to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_SDMMC_REGDEBUG
static void lpc43_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  mcinfo("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/****************************************************************************
 * Name: lpc43_setclock
 *
 * Description:
 *   Define the new clock frequency
 *
 * Input Parameters:
 *   clkdiv - A new division value to generate the needed frequency.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void lpc43_setclock(uint32_t clkdiv)
{
  mcinfo("clkdiv=%08lx\n", (unsigned long)clkdiv);

  /* Disable the clock before setting frequency */

  lpc43_sdcard_clock(false);

  /* Use the Divider0 */

  lpc43_putreg(SDMMC_CLKSRC_CLKDIV0, LPC43_SDMMC_CLKSRC);

  /* Inform CIU */

  lpc43_ciu_sendcmd(SDMMC_CMD_UPDCLOCK | SDMMC_CMD_WAITPREV, 0);

  /* Set Divider0 to desired value */

  lpc43_putreg(clkdiv, LPC43_SDMMC_CLKDIV);

  /* Inform CIU */

  lpc43_ciu_sendcmd(SDMMC_CMD_UPDCLOCK | SDMMC_CMD_WAITPREV, 0);

  /* Enable the clock */

  lpc43_sdcard_clock(true);

  /* Inform CIU */

  lpc43_ciu_sendcmd(SDMMC_CMD_UPDCLOCK | SDMMC_CMD_WAITPREV, 0);
}

/****************************************************************************
 * Name: lpc43_sdcard_clock
 *
 * Description: Enable/Disable the SDCard clock
 *
 * Input Parameters:
 *   enable - False = clock disabled; True = clock enabled.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void lpc43_sdcard_clock(bool enable)
{
  if (enable)
    {
      lpc43_putreg(SDMMC_CLKENA_ENABLE, LPC43_SDMMC_CLKENA);
    }
  else
    {
      lpc43_putreg(0, LPC43_SDMMC_CLKENA);
    }
}

/****************************************************************************
 * Name: lpc43_ciu_sendcmd
 *
 * Description:
 *   Function to send command to Card interface unit (CIU)
 *
 * Input Parameters:
 *   cmd - The command to be executed
 *   arg - The argument to use with the command.
 *
 * Returned Value:
 *   Returns zero on success.  One will be returned on a timeout.
 *
 ****************************************************************************/

static int lpc43_ciu_sendcmd(uint32_t cmd, uint32_t arg)
{
  clock_t watchtime;

  mcinfo("cmd=%04lx arg=%04lx\n", (unsigned long)cmd, (unsigned long)arg);
  DEBUGASSERT((lpc43_getreg(LPC43_SDMMC_CMD) & SDMMC_CMD_STARTCMD) == 0);

  /* Set command arg reg */

  lpc43_putreg(arg, LPC43_SDMMC_CMDARG);
  lpc43_putreg(SDMMC_CMD_STARTCMD | cmd, LPC43_SDMMC_CMD);

  /* Poll until command is accepted by the CIU, or we timeout */

  watchtime = clock_systime_ticks();

  while ((lpc43_getreg(LPC43_SDMMC_CMD) & SDMMC_CMD_STARTCMD) != 0)
    {
      if (watchtime - clock_systime_ticks() > SDCARD_CMDTIMEOUT)
        {
          mcerr("TMO Timed out (%08X)\n",
                lpc43_getreg(LPC43_SDMMC_CMD));
          return 1;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: lpc43_enable_ints
 *
 * Description:
 *   Enable/disable SD card interrupts per functional settings.
 *
 * Input Parameters:
 *   priv - A reference to the SD card device state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_enable_ints(struct lpc43_dev_s *priv)
{
  uint32_t regval;

#ifdef CONFIG_LPC43_SDMMC_DMA
  mcinfo("waitmask=%04lx xfrmask=%04lx dmamask=%04lx RINTSTS=%08lx\n",
         (unsigned long)priv->waitmask, (unsigned long)priv->xfrmask,
         (unsigned long)priv->dmamask,
         (unsigned long)lpc43_getreg(LPC43_SDMMC_RINTSTS));

  /* Enable DMA-related interrupts */

  lpc43_putreg(priv->dmamask, LPC43_SDMMC_IDINTEN);

#else
  mcinfo("waitmask=%04lx xfrmask=%04lx RINTSTS=%08lx\n",
         (unsigned long)priv->waitmask, (unsigned long)priv->xfrmask,
         (unsigned long)lpc43_getreg(LPC43_SDMMC_RINTSTS));
#endif

  /* Enable SDMMC interrupts */

  regval = priv->xfrmask | priv->waitmask | SDCARD_INT_CDET;
  lpc43_putreg(regval, LPC43_SDMMC_INTMASK);
}

/****************************************************************************
 * Name: lpc43_disable_allints
 *
 * Description:
 *   Disable all SD card interrupts.
 *
 * Input Parameters:
 *   priv - A reference to the SD card device state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_disable_allints(struct lpc43_dev_s *priv)
{
#ifdef CONFIG_LPC43_SDMMC_DMA
  /* Disable DMA-related interrupts */

  lpc43_putreg(0, LPC43_SDMMC_IDINTEN);
  priv->dmamask = 0;
#endif

  /* Disable all SDMMC interrupts (except card detect) */

  lpc43_putreg(SDCARD_INT_CDET, LPC43_SDMMC_INTMASK);
  priv->waitmask = 0;
  priv->xfrmask  = 0;
}

/****************************************************************************
 * Name: lpc43_config_waitints
 *
 * Description:
 *   Enable/disable SD card interrupts needed to support the wait function
 *
 * Input Parameters:
 *   priv       - A reference to the SD card device state structure
 *   waitmask   - The set of bits in the SD card INTMASK register to set
 *   waitevents - Waited for events
 *   wkupevent  - Wake-up events
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_config_waitints(struct lpc43_dev_s *priv,
                                  uint32_t waitmask,
                                  sdio_eventset_t waitevents,
                                  sdio_eventset_t wkupevent)
{
  irqstate_t flags;

  mcinfo("waitevents=%04x wkupevent=%04x\n",
         (unsigned)waitevents, (unsigned)wkupevent);

  /* Save all of the data and set the new interrupt mask in one, atomic
   * operation.
   */

  flags            = enter_critical_section();
  priv->waitevents = waitevents;
  priv->wkupevent  = wkupevent;
  priv->waitmask   = waitmask;

  lpc43_enable_ints(priv);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc43_config_xfrints
 *
 * Description:
 *   Enable SD card interrupts needed to support the data transfer event
 *
 * Input Parameters:
 *   priv    - A reference to the SD card device state structure
 *   xfrmask - The set of bits in the SD card MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_config_xfrints(struct lpc43_dev_s *priv, uint32_t xfrmask)
{
  irqstate_t flags;
  flags = enter_critical_section();

  priv->xfrmask = xfrmask;
  lpc43_enable_ints(priv);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpc43_config_dmaints
 *
 * Description:
 *   Enable DMA transfer interrupts
 *
 * Input Parameters:
 *   priv    - A reference to the SD card device state structure
 *   dmamask - The set of bits in the SD card MASK register to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_SDMMC_DMA
static void lpc43_config_dmaints(struct lpc43_dev_s *priv, uint32_t xfrmask,
                                 uint32_t dmamask)
{
  irqstate_t flags;
  flags = enter_critical_section();

  priv->xfrmask = xfrmask;
  priv->dmamask = dmamask;
  lpc43_enable_ints(priv);

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: lpc43_eventtimeout
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

static void lpc43_eventtimeout(wdparm_t arg)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)arg;

  mcinfo("argc=%d, arg=%08lx\n", argc, (unsigned long)arg);

  /* There is always race conditions with timer expirations. */

  DEBUGASSERT((priv->waitevents & SDIOWAIT_TIMEOUT) != 0 ||
              priv->wkupevent != 0);

  /* Is a data transfer complete event expected? */

  if ((priv->waitevents & SDIOWAIT_TIMEOUT) != 0)
    {
      /* Yes.. wake up any waiting threads */

      lpc43_endwait(priv, SDIOWAIT_TIMEOUT);
      mcerr("ERROR: Timeout: remaining: %d\n", priv->remaining);
    }
}

/****************************************************************************
 * Name: lpc43_endwait
 *
 * Description:
 *   Wake up a waiting thread if the waited-for event has occurred.
 *
 * Input Parameters:
 *   priv      - An instance of the SD card device interface
 *   wkupevent - The event that caused the wait to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void lpc43_endwait(struct lpc43_dev_s *priv,
                          sdio_eventset_t wkupevent)
{
  mcinfo("wkupevent=%04x\n", (unsigned)wkupevent);

  /* Cancel the watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Disable event-related interrupts */

  lpc43_config_waitints(priv, 0, 0, wkupevent);

  /* Wake up the waiting thread */

  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: lpc43_endtransfer
 *
 * Description:
 *   Terminate a transfer with the provided status.  This function is called
 *   only from the SD card interrupt handler when end-of-transfer conditions
 *   are detected.
 *
 * Input Parameters:
 *   priv   - An instance of the SD card device interface
 *   wkupevent - The event that caused the transfer to end
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void lpc43_endtransfer(struct lpc43_dev_s *priv,
                              sdio_eventset_t wkupevent)
{
  mcinfo("wkupevent=%04x\n", (unsigned)wkupevent);

  /* Disable all transfer related interrupts */

  lpc43_config_xfrints(priv, 0);

  /* Clearing pending interrupt status on all transfer related interrupts */

  lpc43_putreg(priv->waitmask, LPC43_SDMMC_RINTSTS);

  /* Mark the transfer finished */

  priv->remaining = 0;

  /* Is a thread wait for these data transfer complete events? */

  if ((priv->waitevents & wkupevent) != 0)
    {
      /* Yes.. wake up any waiting threads */

      lpc43_endwait(priv, wkupevent);
    }
}

/****************************************************************************
 * Name: lpc43_sdmmc_interrupt
 *
 * Description:
 *   SD card interrupt handler
 *
 * Input Parameters:
 *   dev - An instance of the SD card device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int lpc43_sdmmc_interrupt(int irq, void *context, void *arg)
{
  struct lpc43_dev_s *priv = &g_scard_dev;
  uint32_t enabled;
  uint32_t pending;

  /* Loop while there are pending interrupts.  Check the SD card status
   * register.  Mask out all bits that don't correspond to enabled
   * interrupts.  (This depends on the fact that bits are ordered
   * the same in both the STA and MASK register).  If there are non-zero
   * bits remaining, then we have work to do here.
   */

  while ((enabled = lpc43_getreg(LPC43_SDMMC_MINTSTS)) != 0)
    {
      /* Clear pending status */

      lpc43_putreg(enabled, LPC43_SDMMC_RINTSTS);

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
      /* Handle in card detection events ************************************/

      if ((enabled & SDMMC_INT_CDET) != 0)
        {
          sdio_statset_t cdstatus;

          /* Update card status */

          cdstatus = priv->cdstatus;
          if ((lpc43_getreg(LPC43_SDMMC_CDETECT) &
              SDMMC_CDETECT_NOTPRESENT) == 0)
            {
              priv->cdstatus |= SDIO_STATUS_PRESENT;

#ifdef CONFIG_MMCSD_HAVE_WRITEPROTECT
              if ((lpc43_getreg(LPC43_SDMMC_WRTPRT) &
                  SDMMC_WRTPRT_PROTECTED) != 0)
                {
                  priv->cdstatus |= SDIO_STATUS_WRPROTECTED;
                }
              else
#endif
                {
                  priv->cdstatus &= ~SDIO_STATUS_WRPROTECTED;
                }

#ifdef CONFIG_LPC43_SDMMC_PWRCTRL
              /* Enable/ power to the SD card */

              lpc43_putreg(SDMMC_PWREN, LPC43_SDMMC_PWREN);
#endif
            }
          else
            {
              priv->cdstatus &=
                ~(SDIO_STATUS_PRESENT | SDIO_STATUS_WRPROTECTED);

#ifdef CONFIG_LPC43_SDMMC_PWRCTRL
              /* Disable power to the SD card */

              lpc43_putreg(0, LPC43_SDMMC_PWREN);
#endif
            }

          mcinfo("cdstatus OLD: %02x NEW: %02x\n", cdstatus, priv->cdstatus);

          /* Perform any requested callback if the status has changed */

          if (cdstatus != priv->cdstatus)
            {
              lpc43_callback(priv);
            }
        }
#endif

      /* Handle data transfer events ****************************************/

      pending = enabled & priv->xfrmask;
      if (pending != 0)
        {
          /* Handle data request events */

          if ((pending & SDMMC_INT_TXDR) != 0)
            {
              uint32_t status;

              /* Transfer data to the TX FIFO */

              DEBUGASSERT(priv->wrdir);

              for (status = lpc43_getreg(LPC43_SDMMC_STATUS);
                   (status & SDMMC_STATUS_FIFOFULL) == 0 &&
                   priv->remaining > 0;
                   status = lpc43_getreg(LPC43_SDMMC_STATUS))
                {
                  lpc43_putreg(*priv->buffer, LPC43_SDMMC_DATA);
                  priv->buffer++;
                  priv->remaining -= 4;
                }
            }
          else if ((pending & SDMMC_INT_RXDR) != 0)
            {
              uint32_t status;

              /* Transfer data from the RX FIFO */

              DEBUGASSERT(!priv->wrdir);

              for (status = lpc43_getreg(LPC43_SDMMC_STATUS);
                   (status & SDMMC_STATUS_FIFOEMPTY) == 0 &&
                   priv->remaining > 0;
                   status = lpc43_getreg(LPC43_SDMMC_STATUS))
                {
                  *priv->buffer = lpc43_getreg(LPC43_SDMMC_DATA);
                  priv->buffer++;
                  priv->remaining -= 4;
                }
            }

          /* Check for transfer errors */

          /* Handle data block send/receive CRC failure */

          if ((pending & SDMMC_INT_DCRC) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data CRC failure, pending=%08x remaining: %d\n",
                    pending, priv->remaining);

              lpc43_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle data timeout error */

          else if ((pending & SDMMC_INT_DRTO) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Data timeout, pending=%08x remaining: %d\n",
                    pending, priv->remaining);

              lpc43_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_TIMEOUT);
            }

          /* Handle RX FIFO overrun error */

          else if ((pending & SDMMC_INT_FRUN) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: RX FIFO overrun, pending=%08x remaining: %d\n",
                    pending, priv->remaining);

              lpc43_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle TX FIFO underrun error */

          else if ((pending & SDMMC_INT_FRUN) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: TX FIFO underrun, pending=%08x remaining: %d\n",
                    pending, priv->remaining);

              lpc43_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle start bit error */

          else if ((pending & SDMMC_INT_SBE) != 0)
            {
              /* Terminate the transfer with an error */

              mcerr("ERROR: Start bit, pending=%08x remaining: %d\n",
                    pending, priv->remaining);

              lpc43_endtransfer(priv,
                                SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
            }

          /* Handle data end events.  Note that RXDR may accompany DTO, DTO
           * will be set on received while there is still data in the FIFO.
           * So for the case of receiving, we don't actually even enable the
           * DTO interrupt.
           */

          else if ((pending & SDMMC_INT_DTO) != 0)
            {
              /* Finish the transfer */

              lpc43_endtransfer(priv, SDIOWAIT_TRANSFERDONE);
            }
        }

      /* Handle wait events *************************************************/

      pending = enabled & priv->waitmask;
      if (pending != 0)
        {
          /* Is this a response error event? */

          if ((pending & SDCARD_INT_RESPERR) != 0)
            {
              /* If response errors are enabled, then we must certainly be
               * waiting for a response.
               */

              DEBUGASSERT((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0);

              /* Wake the thread up */

              mcerr("ERROR: Response error, pending=%08x\n", pending);
              lpc43_endwait(priv, SDIOWAIT_RESPONSEDONE | SDIOWAIT_ERROR);
            }

          /* Is this a command (plus response) completion event? */

          else if ((pending & SDMMC_INT_CDONE) != 0)
            {
              /* Yes.. Is their a thread waiting for response done? */

              if ((priv->waitevents & SDIOWAIT_RESPONSEDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  lpc43_endwait(priv, SDIOWAIT_RESPONSEDONE);
                }

              /* NO.. Is their a thread waiting for command done? */

              else if ((priv->waitevents & SDIOWAIT_CMDDONE) != 0)
                {
                  /* Yes.. wake the thread up */

                  lpc43_endwait(priv, SDIOWAIT_CMDDONE);
                }
            }
        }
    }

#ifdef CONFIG_LPC43_SDMMC_DMA
  /* DMA error events *******************************************************/

  pending = lpc43_getreg(LPC43_SDMMC_IDSTS);
  if ((pending & priv->dmamask) != 0)
    {
      mcerr("ERROR: IDTS=%08lx\n", (unsigned long)pending);

      /* Clear the pending interrupts */

      lpc43_putreg(pending, LPC43_SDMMC_IDSTS);

      /* Abort the transfer */

      lpc43_endtransfer(priv, SDIOWAIT_TRANSFERDONE | SDIOWAIT_ERROR);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: lpc43_lock
 *
 * Description:
 *   Locks the bus. Function calls low-level multiplexed bus routines to
 *   resolve bus requests and acknowledgment issues.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   lock   - TRUE to lock, FALSE to unlock.
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SDIO_MUXBUS
static int lpc43_lock(struct sdio_dev_s *dev, bool lock)
{
  /* Single SD card instance so there is only one possibility.  The multiplex
   * bus is part of board support package.
   */

  lpc43_muxbus_sdio_lock(lock);
  return OK;
}
#endif

/****************************************************************************
 * Name: lpc43_reset
 *
 * Description:
 *   Reset the SD card controller.  Undo all setup and initialization.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_reset(struct sdio_dev_s *dev)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  mcinfo("Resetting...\n");

  flags = enter_critical_section();

  /* Reset DMA controller internal registers. */

  lpc43_putreg(SDMMC_BMOD_SWR, LPC43_SDMMC_BMOD);

  /* Reset all blocks */

  lpc43_putreg(SDMMC_CTRL_CNTLRRESET | SDMMC_CTRL_FIFORESET |
               SDMMC_CTRL_DMARESET, LPC43_SDMMC_CTRL);

  while ((lpc43_getreg(LPC43_SDMMC_CTRL) &
          (SDMMC_CTRL_CNTLRRESET | SDMMC_CTRL_FIFORESET |
           SDMMC_CTRL_DMARESET)) != 0)
    {
    }

  /* Reset data */

  priv->waitevents = 0;      /* Set of events to be waited for */
  priv->waitmask   = 0;      /* Interrupt enables for event waiting */
  priv->wkupevent  = 0;      /* The event that caused the wakeup */

  wd_cancel(&priv->waitwdog); /* Cancel any timeouts */

  /* Interrupt mode data transfer support */

  priv->buffer     = 0;      /* Address of current R/W buffer */
  priv->remaining  = 0;      /* Number of bytes remaining in the transfer */
  priv->xfrmask    = 0;      /* Interrupt enables for data transfer */
#ifdef CONFIG_LPC43_SDMMC_DMA
  priv->dmamask    = 0;      /* Interrupt enables for DMA transfer */
#endif

  /* DMA data transfer support */

  priv->widebus    = true;   /* Required for DMA support */
  priv->cdstatus   = 0;      /* Card status is unknown */

#ifdef CONFIG_LPC43_SDMMC_DMA
  priv->dmamode    = false;  /* true: DMA mode transfer */
#endif

  /* Select 1-bit wide bus */

  lpc43_putreg(SDMMC_CTYPE_WIDTH1, LPC43_SDMMC_CTYPE);

  /* Enable interrupts */

  regval  = lpc43_getreg(LPC43_SDMMC_CTRL);
  regval |= SDMMC_CTRL_INTENABLE;
  lpc43_putreg(regval, LPC43_SDMMC_CTRL);

  /* Disable Interrupts except for card detection. */

  lpc43_putreg(SDCARD_INT_CDET, LPC43_SDMMC_INTMASK);

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
  /* Set the card debounce time.  Number of host clocks (SD_CLK) used by
   * debounce filter logic for card detect.  typical debounce time is 5-25
   * ms.
   */

  lpc43_putreg(DEBOUNCE_TICKS, LPC43_SDMMC_DEBNCE);
#endif

  /* Clear to Interrupts */

  lpc43_putreg(0xffffffff, LPC43_SDMMC_RINTSTS);

  /* Define MAX Timeout */

  lpc43_putreg(0x7fffffff, LPC43_SDMMC_TMOUT);

  /* Disable clock to CIU (needs latch) */

  lpc43_putreg(0, LPC43_SDMMC_CLKENA);
  leave_critical_section(flags);

#if defined(CONFIG_LPC43_SDMMC_PWRCTRL) && !defined(CONFIG_MMCSD_HAVE_CARDDETECT)
  /* Enable power to the SD card */

  lpc43_putreg(SDMMC_PWREN, LPC43_SDMMC_PWREN);
#endif
}

/****************************************************************************
 * Name: lpc43_capabilities
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

static sdio_capset_t lpc43_capabilities(struct sdio_dev_s *dev)
{
  sdio_capset_t caps = 0;

  caps |= SDIO_CAPS_DMABEFOREWRITE;

#ifdef CONFIG_SDIO_WIDTH_D1_ONLY
  caps |= SDIO_CAPS_1BIT_ONLY;
#endif
#ifdef CONFIG_LPC43_SDMMC_DMA
  caps |= SDIO_CAPS_DMASUPPORTED;
#endif

  return caps;
}

/****************************************************************************
 * Name: lpc43_status
 *
 * Description:
 *   Get SD card status.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *
 * Returned Value:
 *   Returns a bitset of status values (see lpc43_status_* defines)
 *
 ****************************************************************************/

static sdio_statset_t lpc43_status(struct sdio_dev_s *dev)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;

#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
  if ((lpc43_getreg(LPC43_SDMMC_CDETECT) & SDMMC_CDETECT_NOTPRESENT) == 0)
    {
      priv->cdstatus |= SDIO_STATUS_PRESENT;
    }
  else
    {
      priv->cdstatus &= ~SDIO_STATUS_PRESENT;
    }
#endif

  mcinfo("cdstatus=%02x\n", priv->cdstatus);

  return priv->cdstatus;
}

/****************************************************************************
 * Name: lpc43_widebus
 *
 * Description:
 *   Called after change in Bus width has been selected (via ACMD6).  Most
 *   controllers will need to perform some special operations to work
 *   correctly in the new bus mode.
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *   wide - true: wide bus (4-bit) bus mode enabled
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_widebus(struct sdio_dev_s *dev, bool wide)
{
  mcinfo("wide=%d\n", wide);
}

/****************************************************************************
 * Name: lpc43_clock
 *
 * Description:
 *   Enable/disable SD card clocking
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *   rate - Specifies the clocking to use (see enum sdio_clock_e)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_clock(struct sdio_dev_s *dev, enum sdio_clock_e rate)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  uint8_t clkdiv;
  uint8_t ctype;
  bool enabled = false;
  bool widebus = false;

  switch (rate)
    {
      /* Disable clocking (with default ID mode divisor) */

      default:
      case CLOCK_SDIO_DISABLED:
        clkdiv  = SDMMC_CLKDIV0(BOARD_CLKDIV_INIT);
        ctype   = SDCARD_BUS_D1;
        enabled = false;
        widebus = false;
        return;
        break;

      /* Enable in initial ID mode clocking (<400KHz) */

      case CLOCK_IDMODE:
        clkdiv  = SDMMC_CLKDIV0(BOARD_CLKDIV_INIT);
        ctype   = SDCARD_BUS_D1;
        enabled = true;
        widebus = false;
        break;

      /* Enable in MMC normal operation clocking */

      case CLOCK_MMC_TRANSFER:
        clkdiv  = SDMMC_CLKDIV0(BOARD_CLKDIV_MMCXFR);
        ctype   = SDCARD_BUS_D1;
        enabled = true;
        widebus = false;
        break;

      /* SD normal operation clocking (wide 4-bit mode) */

      case CLOCK_SD_TRANSFER_4BIT:
#ifndef CONFIG_SDIO_WIDTH_D1_ONLY
        clkdiv  = SDMMC_CLKDIV0(BOARD_CLKDIV_SDWIDEXFR);
        ctype   = SDCARD_BUS_D4;
        enabled = true;
        widebus = true;
        break;
#endif

      /* SD normal operation clocking (narrow 1-bit mode) */

      case CLOCK_SD_TRANSFER_1BIT:
        clkdiv  = SDMMC_CLKDIV0(BOARD_CLKDIV_SDXFR);
        ctype   = SDCARD_BUS_D1;
        enabled = true;
        widebus = false;
        break;
    }

  /* Setup the card bus width */

  mcinfo("widebus=%d\n", widebus);

  priv->widebus = widebus;
  lpc43_putreg(ctype, LPC43_SDMMC_CTYPE);

  /* Set the new clock frequency division */

  lpc43_setclock(clkdiv);

  /* Enable the new clock */

  lpc43_sdcard_clock(enabled);
}

/****************************************************************************
 * Name: lpc43_attach
 *
 * Description:
 *   Attach and prepare interrupts
 *
 * Input Parameters:
 *   dev - An instance of the SD card device interface
 *
 * Returned Value:
 *   OK on success; A negated errno on failure.
 *
 ****************************************************************************/

static int lpc43_attach(struct sdio_dev_s *dev)
{
  int ret;
  uint32_t regval;

  mcinfo("Attaching..\n");

  /* Attach the SD card interrupt handler */

  ret = irq_attach(LPC43M4_IRQ_SDIO, lpc43_sdmmc_interrupt, NULL);
  if (ret == OK)
    {
      /* Disable all interrupts at the SD card controller and clear static
       * interrupt flags
       */

      lpc43_putreg(0, LPC43_SDMMC_INTMASK);
      lpc43_putreg(SDMMC_INT_ALL, LPC43_SDMMC_RINTSTS);

      /* Enable Interrupts to happen when the INTMASK is activated */

      regval  = lpc43_getreg(LPC43_SDMMC_CTRL);
      regval |= SDMMC_CTRL_INTENABLE;
      lpc43_putreg(regval, LPC43_SDMMC_CTRL);

      /* Enable card detection interrupts */

      lpc43_putreg(SDCARD_INT_CDET, LPC43_SDMMC_INTMASK);

      /* Enable SD card interrupts at the NVIC.  They can now be enabled at
       * the SD card controller as needed.
       */

      up_enable_irq(LPC43M4_IRQ_SDIO);
    }

  return ret;
}

/****************************************************************************
 * Name: lpc43_sendcmd
 *
 * Description:
 *   Send the SD card command
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *   cmd  - The command to send (32-bits, encoded)
 *   arg  - 32-bit argument required with some commands
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int lpc43_sendcmd(struct sdio_dev_s *dev, uint32_t cmd,
                         uint32_t arg)
{
  uint32_t regval = 0;
  uint32_t cmdidx;

  mcinfo("cmd=%04x arg=%04x\n", cmd, arg);

  /* The CMD0 needs the SENDINIT CMD */

  if (cmd == 0)
    {
      regval |= SDMMC_CMD_SENDINIT;
    }

  /* Is this a Read/Write Transfer Command ? */

  if ((cmd & MMCSD_WRDATAXFR) == MMCSD_WRDATAXFR)
    {
      regval |= SDMMC_CMD_DATAXFREXPTD | SDMMC_CMD_WRITE |
                SDMMC_CMD_WAITPREV;
    }
  else if ((cmd & MMCSD_RDDATAXFR) == MMCSD_RDDATAXFR)
    {
      regval |= SDMMC_CMD_DATAXFREXPTD | SDMMC_CMD_WAITPREV;
    }

  /* Set WAITRESP bits */

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      regval |= SDMMC_CMD_NORESPONSE;
      break;

    case MMCSD_R1B_RESPONSE:
      regval |= SDMMC_CMD_WAITPREV;
    case MMCSD_R1_RESPONSE:
    case MMCSD_R3_RESPONSE:
    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
    case MMCSD_R6_RESPONSE:
    case MMCSD_R7_RESPONSE:
      regval |= SDMMC_CMD_SHORTRESPONSE;
      break;

    case MMCSD_R2_RESPONSE:
      regval |= SDMMC_CMD_LONGRESPONSE;
      break;
    }

  /* Set the command index */

  cmdidx  = (cmd & MMCSD_CMDIDX_MASK) >> MMCSD_CMDIDX_SHIFT;
  regval |= cmdidx;

  mcinfo("cmd: %04x arg: %04x regval: %08x\n", cmd, arg, regval);

  /* Write the SD card CMD */

  lpc43_putreg(SDCARD_RESPDONE_CLEAR | SDCARD_CMDDONE_CLEAR,
               LPC43_SDMMC_RINTSTS);
  lpc43_ciu_sendcmd(regval, arg);

  return OK;
}

/****************************************************************************
 * Name: lpc43_blocksetup
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
static void lpc43_blocksetup(struct sdio_dev_s *dev,
                             unsigned int blocklen, unsigned int nblocks)
{
  mcinfo("blocklen=%ld, total transfer=%ld (%ld blocks)\n",
         blocklen, blocklen * nblocks, nblocks);

  /* Configure block size for next transfer */

  lpc43_putreg(blocklen, LPC43_SDMMC_BLKSIZ);
  lpc43_putreg(blocklen * nblocks, LPC43_SDMMC_BYTCNT);
}
#endif

/****************************************************************************
 * Name: lpc43_recvsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card in non-
 *   DMA (interrupt driven mode).  This method will do whatever controller
 *   setup is necessary.  This would be called for SD memory just BEFORE
 *   sending CMD13 (SEND_STATUS), CMD17 (READ_SINGLE_BLOCK), CMD18
 *   (READ_MULTIPLE_BLOCKS), ACMD51 (SEND_SCR), etc.  Normally,
 *   SDCARD_WAITEVENT will be called to receive the indication that the
 *   transfer is complete.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   buffer - Address of the buffer in which to receive the data
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc43_recvsetup(struct sdio_dev_s *dev, uint8_t *buffer,
                           size_t nbytes)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
#ifdef CONFIG_LPC43_SDMMC_DMA
  uint32_t regval;
#endif

  mcinfo("nbytes=%ld\n", (long) nbytes);

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
  priv->wrdir     = false;
#ifdef CONFIG_LPC43_SDMMC_DMA
  priv->dmamode   = false;
#endif

  /* Configure the FIFO so that we will receive the RXDR interrupt whenever
   * there are more than 1 words (at least 8 bytes) in the RX FIFO.
   */

  lpc43_putreg(SDMMC_FIFOTH_RXWMARK(1), LPC43_SDMMC_FIFOTH);

#ifdef CONFIG_LPC43_SDMMC_DMA
  /* Make sure that internal DMA is disabled */

  lpc43_putreg(0, LPC43_SDMMC_BMOD);

  regval  = lpc43_getreg(LPC43_SDMMC_CTRL);
  regval &= ~SDMMC_CTRL_INTDMA;
  lpc43_putreg(regval, LPC43_SDMMC_CTRL);
#endif

  /* Flush ints before we start */

  lpc43_putreg(SDCARD_TRANSFER_ALL, LPC43_SDMMC_RINTSTS);

  /* Configure the transfer interrupts */

  lpc43_config_xfrints(priv, SDCARD_RECV_MASK);
  return OK;
}

/****************************************************************************
 * Name: lpc43_sendsetup
 *
 * Description:
 *   Setup hardware in preparation for data transfer from the card.  This
 *   method will do whatever controller setup is necessary.  This would be
 *   called for SD memory just AFTER sending CMD24 (WRITE_BLOCK), CMD25
 *   (WRITE_MULTIPLE_BLOCK), ... and before SDCARD_SENDDATA is called.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   buffer - Address of the buffer containing the data to send
 *   nbytes - The number of bytes in the transfer
 *
 * Returned Value:
 *   Number of bytes sent on success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc43_sendsetup(struct sdio_dev_s *dev,
                           const uint8_t *buffer, size_t nbytes)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
#ifdef CONFIG_LPC43_SDMMC_DMA
  uint32_t regval;
#endif

  mcinfo("nbytes=%ld\n", (long)nbytes);

  DEBUGASSERT(priv != NULL && buffer != NULL && nbytes > 0);
  DEBUGASSERT(((uint32_t)buffer & 3) == 0);

  /* Save the source buffer information for use by the interrupt handler */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = nbytes;
  priv->wrdir     = true;
#ifdef CONFIG_LPC43_SDMMC_DMA
  priv->dmamode   = false;
#endif

  /* Configure the FIFO so that we will receive the TXDR interrupt whenever
   * there the TX FIFO is at least half empty.
   */

  lpc43_putreg(SDMMC_FIFOTH_TXWMARK(LPC43_TXFIFO_DEPTH / 2),
               LPC43_SDMMC_FIFOTH);

#ifdef CONFIG_LPC43_SDMMC_DMA
  /* Make sure that internal DMA is disabled */

  lpc43_putreg(0, LPC43_SDMMC_BMOD);

  regval  = lpc43_getreg(LPC43_SDMMC_CTRL);
  regval &= ~SDMMC_CTRL_INTDMA;
  lpc43_putreg(regval, LPC43_SDMMC_CTRL);
#endif

  /* Flush ints before we start */

  lpc43_putreg(SDCARD_TRANSFER_ALL, LPC43_SDMMC_RINTSTS);

  /* Configure the transfer interrupts */

  lpc43_config_xfrints(priv, SDCARD_SEND_MASK);
  return OK;
}

/****************************************************************************
 * Name: lpc43_cancel
 *
 * Description:
 *   Cancel the data transfer setup of SDCARD_RECVSETUP, SDCARD_SENDSETUP,
 *   SDCARD_DMARECVSETUP or SDCARD_DMASENDSETUP.  This must be called to
 *   cancel the data transfer setup if, for some reason, you cannot perform
 *   the transfer.
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc43_cancel(struct sdio_dev_s *dev)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;

  mcinfo("Cancelling..\n");

  /* Disable all transfer- and event- related interrupts */

  lpc43_disable_allints(priv);

  /* Clearing pending interrupt status on all transfer- and event- related
   * interrupts
   */

  lpc43_putreg(SDCARD_WAITALL_CLEAR, LPC43_SDMMC_RINTSTS);

  /* Cancel any watchdog timeout */

  wd_cancel(&priv->waitwdog);

  /* Mark no transfer in progress */

  priv->remaining = 0;
  return OK;
}

/****************************************************************************
 * Name: lpc43_waitresponse
 *
 * Description:
 *   Poll-wait for the response to the last command to be ready.
 *
 * Input Parameters:
 *   dev  - An instance of the SD card device interface
 *   cmd  - The command that was sent.  See 32-bit command definitions above.
 *
 * Returned Value:
 *   OK is success; a negated errno on failure
 *
 ****************************************************************************/

static int lpc43_waitresponse(struct sdio_dev_s *dev, uint32_t cmd)
{
  volatile int32_t timeout;
  clock_t watchtime;
  uint32_t events;

  mcinfo("cmd=%04x\n", cmd);

  switch (cmd & MMCSD_RESPONSE_MASK)
    {
    case MMCSD_NO_RESPONSE:
      events  = SDCARD_CMDDONE_STA;
      timeout = SDCARD_CMDTIMEOUT;
      break;

    case MMCSD_R1_RESPONSE:
    case MMCSD_R1B_RESPONSE:
    case MMCSD_R2_RESPONSE:
    case MMCSD_R6_RESPONSE:
      events  = (SDCARD_CMDDONE_STA | SDCARD_RESPDONE_STA);
      timeout = SDCARD_LONGTIMEOUT;
      break;

    case MMCSD_R4_RESPONSE:
    case MMCSD_R5_RESPONSE:
      return -ENOSYS;

    case MMCSD_R3_RESPONSE:
    case MMCSD_R7_RESPONSE:
      events  = (SDCARD_CMDDONE_STA | SDCARD_RESPDONE_STA);
      timeout = SDCARD_CMDTIMEOUT;
      break;

    default:
      return -EINVAL;
    }

  mcinfo("cmd: %04x events: %04x STATUS: %08x RINTSTS: %08x\n",
         cmd, events, lpc43_getreg(LPC43_SDMMC_STATUS),
         lpc43_getreg(LPC43_SDMMC_RINTSTS));

  /* Then wait for the response (or timeout or error) */

  watchtime = clock_systime_ticks();
  while ((lpc43_getreg(LPC43_SDMMC_RINTSTS) & events) != events)
    {
      if (clock_systime_ticks() - watchtime > timeout)
        {
          mcerr("ERROR: Timeout cmd: %04x events: %04x STA: %08x "
                "RINTSTS: %08x\n",
                cmd, events, lpc43_getreg(LPC43_SDMMC_STATUS),
                lpc43_getreg(LPC43_SDMMC_RINTSTS));

          return -ETIMEDOUT;
        }
      else if ((lpc43_getreg(LPC43_SDMMC_RINTSTS) & SDCARD_INT_RESPERR) != 0)
        {
          mcerr("ERROR: SDMMC failure cmd: %04x events: %04x STA: %08x "
                "RINTSTS: %08x\n",
                cmd, events, lpc43_getreg(LPC43_SDMMC_STATUS),
                lpc43_getreg(LPC43_SDMMC_RINTSTS));

          return -EIO;
        }
    }

  lpc43_putreg(SDCARD_CMDDONE_CLEAR, LPC43_SDMMC_RINTSTS);
  return OK;
}

/****************************************************************************
 * Name: lpc43_recv*
 *
 * Description:
 *   Receive response to SD card command.  Only the critical payload is
 *   returned -- that is 32 bits for 48 bit status and 128 bits for 136 bit
 *   status.  The driver implementation should verify the correctness of
 *   the remaining, non-returned bits (CRCs, CMD index, etc.).
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
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

static int lpc43_recvshortcrc(struct sdio_dev_s *dev, uint32_t cmd,
                              uint32_t *rshort)
{
  uint32_t regval;

  int ret = OK;

  mcinfo("cmd=%04x\n", cmd);

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

#ifdef CONFIG_DEBUG_FEATURES
  if (!rshort)
    {
      mcerr("ERROR: rshort=NULL\n");
      ret = -EINVAL;
    }

  /* Check that this is the correct response to this command */

  else if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R1B_RESPONSE &&
           (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R6_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%04x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = lpc43_getreg(LPC43_SDMMC_RINTSTS);
      if ((regval & SDMMC_INT_RTO) != 0)
        {
          mcerr("ERROR: Command timeout: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if ((regval & SDMMC_INT_RCRC) != 0)
        {
          mcerr("ERROR: CRC failure: %08x\n", regval);
          ret = -EIO;
        }
    }

  /* Clear all pending message completion events and return the R1/R6
   * response.
   */

  lpc43_putreg(SDCARD_RESPDONE_CLEAR | SDCARD_CMDDONE_CLEAR,
               LPC43_SDMMC_RINTSTS);
  *rshort = lpc43_getreg(LPC43_SDMMC_RESP0);
  mcinfo("CRC=%04x\n", *rshort);

  return ret;
}

static int lpc43_recvlong(struct sdio_dev_s *dev, uint32_t cmd,
                          uint32_t rlong[4])
{
  uint32_t regval;
  int ret = OK;

  mcinfo("cmd=%04x\n", cmd);

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
      mcerr("ERROR: Wrong response CMD=%04x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout or CRC error occurred */

      regval = lpc43_getreg(LPC43_SDMMC_RINTSTS);
      if (regval & SDMMC_INT_RTO)
        {
          mcerr("ERROR: Timeout STA: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
      else if (regval & SDMMC_INT_RCRC)
        {
          mcerr("ERROR: CRC fail STA: %08x\n", regval);
          ret = -EIO;
        }
    }

  /* Return the long response */

  lpc43_putreg(SDCARD_RESPDONE_CLEAR | SDCARD_CMDDONE_CLEAR,
               LPC43_SDMMC_RINTSTS);
  if (rlong)
    {
      rlong[0] = lpc43_getreg(LPC43_SDMMC_RESP3);
      rlong[1] = lpc43_getreg(LPC43_SDMMC_RESP2);
      rlong[2] = lpc43_getreg(LPC43_SDMMC_RESP1);
      rlong[3] = lpc43_getreg(LPC43_SDMMC_RESP0);
    }

  return ret;
}

static int lpc43_recvshort(struct sdio_dev_s *dev, uint32_t cmd,
                           uint32_t *rshort)
{
  uint32_t regval;
  int ret = OK;

  mcinfo("cmd=%04x\n", cmd);

  /* R3  OCR (48-bit)
   *     47        0               Start bit
   *     46        0               Transmission bit (0=from card)
   *     45:40     bit5   - bit0   Reserved
   *     39:8      bit31  - bit0   32-bit OCR register
   *     7:1       bit6   - bit0   Reserved
   *     0         1               End bit
   */

  /* Check that this is the correct response to this command */

#ifdef CONFIG_DEBUG_FEATURES
  if ((cmd & MMCSD_RESPONSE_MASK) != MMCSD_R3_RESPONSE &&
      (cmd & MMCSD_RESPONSE_MASK) != MMCSD_R7_RESPONSE)
    {
      mcerr("ERROR: Wrong response CMD=%04x\n", cmd);
      ret = -EINVAL;
    }
  else
#endif
    {
      /* Check if a timeout occurred (Apparently a CRC error can terminate
       * a good response)
       */

      regval = lpc43_getreg(LPC43_SDMMC_RINTSTS);
      if (regval & SDMMC_INT_RTO)
        {
          mcerr("ERROR: Timeout STA: %08x\n", regval);
          ret = -ETIMEDOUT;
        }
    }

  lpc43_putreg(SDCARD_RESPDONE_CLEAR | SDCARD_CMDDONE_CLEAR,
               LPC43_SDMMC_RINTSTS);
  if (rshort)
    {
      *rshort = lpc43_getreg(LPC43_SDMMC_RESP0);
    }

  return ret;
}

/* MMC responses not supported */

static int lpc43_recvnotimpl(struct sdio_dev_s *dev, uint32_t cmd,
                             uint32_t *rnotimpl)
{
  mcinfo("cmd=%04x\n", cmd);

  lpc43_putreg(SDCARD_RESPDONE_CLEAR | SDCARD_CMDDONE_CLEAR,
               LPC43_SDMMC_RINTSTS);
  return -ENOSYS;
}

/****************************************************************************
 * Name: lpc43_waitenable
 *
 * Description:
 *   Enable/disable of a set of SD card wait events.  This is part of the
 *   the SDCARD_WAITEVENT sequence.  The set of to-be-waited-for events is
 *   configured before calling lpc43_eventwait.  This is done in this way
 *   to help the driver to eliminate race conditions between the command
 *   setup and the subsequent events.
 *
 *   The enabled events persist until either (1) SDCARD_WAITENABLE is called
 *   again specifying a different set of wait events, or (2) SDCARD_EVENTWAIT
 *   returns.
 *
 * Input Parameters:
 *   dev      - An instance of the SD card device interface
 *   eventset - A bitset of events to enable or disable (see SDIOWAIT_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_waitenable(struct sdio_dev_s *dev,
                             sdio_eventset_t eventset, uint32_t timeout)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  uint32_t waitmask;

  mcinfo("eventset=%04x\n", (unsigned int)eventset);
  DEBUGASSERT(priv != NULL);

  /* Disable event-related interrupts */

  lpc43_config_waitints(priv, 0, 0, 0);

  /* Select the interrupt mask that will give us the appropriate wakeup
   * interrupts.
   */

  waitmask = 0;
  if ((eventset & SDIOWAIT_CMDDONE) != 0)
    {
      waitmask |= SDCARD_CMDDONE_MASK;
    }

  if ((eventset & SDIOWAIT_RESPONSEDONE) != 0)
    {
      waitmask |= SDCARD_RESPDONE_MASK;
    }

  if ((eventset & SDIOWAIT_TRANSFERDONE) != 0)
    {
      waitmask |= SDCARD_XFRDONE_MASK;
    }

  /* Enable event-related interrupts */

  lpc43_config_waitints(priv, waitmask, eventset, 0);

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
                       lpc43_eventtimeout, (wdparm_t)priv);
      if (ret < 0)
        {
          mcerr("ERROR: wd_start failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: lpc43_eventwait
 *
 * Description:
 *   Wait for one of the enabled events to occur (or a timeout).  Note that
 *   all events enabled by SDCARD_WAITEVENTS are disabled when
 *   lpc43_eventwait returns.  SDCARD_WAITEVENTS must be called again before
 *   lpc43_eventwait can be used again.
 *
 * Input Parameters:
 *   dev     - An instance of the SD card device interface
 *   timeout - Maximum time in milliseconds to wait.  Zero means immediate
 *             timeout with no wait.  The timeout value is ignored if
 *             SDIOWAIT_TIMEOUT is not included in the waited-for eventset.
 *
 * Returned Value:
 *   Event set containing the event(s) that ended the wait.  Should always
 *   be non-zero.  All events are disabled after the wait concludes.
 *
 ****************************************************************************/

static sdio_eventset_t lpc43_eventwait(struct sdio_dev_s *dev)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  sdio_eventset_t wkupevent = 0;
  irqstate_t flags;
  int ret;

  mcinfo("timeout=%lu\n", (unsigned long)timeout);

  /* There is a race condition here... the event may have completed before
   * we get here.  In this case waitevents will be zero, but wkupevents will
   * be non-zero (and, hopefully, the semaphore count will also be non-zero.
   */

  flags = enter_critical_section();
  DEBUGASSERT(priv->waitevents != 0 || priv->wkupevent != 0);

  /* Loop until the event (or the timeout occurs). Race conditions are
   * avoided by calling lpc43_waitenable prior to triggering the logic that
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
          /* Task canceled.  Cancel the wdog -- assuming it was started and
           * return an SDIO error.
           */

          wd_cancel(&priv->waitwdog);
          leave_critical_section(flags);
          return SDIOWAIT_ERROR;
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

  /* Disable all transfer- and event- related interrupts */

  lpc43_disable_allints(priv);

errout:
  leave_critical_section(flags);
  mcinfo("wkupevent=%04x\n", wkupevent);
  return wkupevent;
}

/****************************************************************************
 * Name: lpc43_callbackenable
 *
 * Description:
 *   Enable/disable of a set of SD card callback events.  This is part of
 *   the SD card callback sequence.  The set of events is configured to
 *   enabled callbacks to the function provided in lpc43_registercallback.
 *
 *   Events are automatically disabled once the callback is performed and no
 *   further callback events will occur until they are again enabled by
 *   calling this method.
 *
 * Input Parameters:
 *   dev      - An instance of the SD card device interface
 *   eventset - A bitset of events to enable or disable (see SDIOMEDIA_*
 *              definitions). 0=disable; 1=enable.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc43_callbackenable(struct sdio_dev_s *dev,
                                 sdio_eventset_t eventset)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;

  mcinfo("eventset: %02x\n", eventset);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = eventset;
  lpc43_callback(priv);
}

/****************************************************************************
 * Name: lpc43_registercallback
 *
 * Description:
 *   Register a callback that that will be invoked on any media status
 *   change.  Callbacks should not be made from interrupt handlers, rather
 *   interrupt level events should be handled by calling back on the work
 *   thread.
 *
 *   When this method is called, all callbacks should be disabled until they
 *   are enabled via a call to SDCARD_CALLBACKENABLE
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

static int lpc43_registercallback(struct sdio_dev_s *dev,
                                  worker_t callback, void *arg)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;

  mcinfo("callback=%p arg=%p\n", callback, arg);

  /* Disable callbacks and register this callback and its argument */

  mcinfo("Register %p(%p)\n", callback, arg);
  DEBUGASSERT(priv != NULL);

  priv->cbevents = 0;
  priv->cbarg    = arg;
  priv->callback = callback;
  return OK;
}

/****************************************************************************
 * Name: lpc43_dmarecvsetup
 *
 * Description:
 *   Setup to perform a read DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For read transfers this may mean
 *   invalidating the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   buffer - The memory to DMA from
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_SDMMC_DMA
static int lpc43_dmarecvsetup(struct sdio_dev_s *dev,
                              uint8_t *buffer, size_t buflen)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  uint32_t regval;
  uint32_t ctrl;
  uint32_t maxs;
  int i;

  /* Don't bother with DMA if the entire transfer will fit in the RX FIFO or
   * if we do not have a 4-bit wide bus.
   */

  DEBUGASSERT(priv != NULL);

  if (buflen <= LPC43_RXFIFO_SIZE || !priv->widebus)
    {
      return lpc43_recvsetup(dev, buffer, buflen);
    }

  mcinfo("buffer=%p buflen=%lu\n", buffer, (unsigned long)buflen, buffer);
  DEBUGASSERT(buffer != NULL && buflen > 0 && ((uint32_t)buffer & 3) == 0);

  /* Reset DMA controller internal registers.  The SWR bit automatically
   * clears in one clock cycle.
   */

  lpc43_putreg(SDMMC_BMOD_SWR, LPC43_SDMMC_BMOD);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->wrdir     = false;
  priv->dmamode   = true;

  /* Reset the FIFO and DMA */

  regval  = lpc43_getreg(LPC43_SDMMC_CTRL);
  regval |= SDMMC_CTRL_FIFORESET | SDMMC_CTRL_DMARESET;
  lpc43_putreg(regval, LPC43_SDMMC_CTRL);

  while ((lpc43_getreg(LPC43_SDMMC_CTRL) & (SDMMC_CTRL_FIFORESET |
                                            SDMMC_CTRL_DMARESET)) != 0)
    {
    }

  /* Configure the FIFO so that we will receive the DMA/FIFO requests
   * whenever there more than than (FIFO_DEPTH/2) - 1 words in the FIFO.
   */

  regval = SDMMC_FIFOTH_RXWMARK(LPC43_RXFIFO_DEPTH / 2 - 1) |
           SDMMC_FIFOTH_DMABURST_4XFRS;
  lpc43_putreg(regval, LPC43_SDMMC_FIFOTH);

  /* Setup DMA list */

  i = 0;
  while (buflen > 0)
    {
      /* Limit size of the transfer to maximum buffer size */

      maxs = buflen;

      if (maxs > MCI_DMADES1_MAXTR)
        {
          maxs = MCI_DMADES1_MAXTR;
        }

      buflen -= maxs;

      /* Set buffer size */

      g_sdmmc_dmadd[i].des1 = MCI_DMADES1_BS1(maxs);

      /* Setup buffer address (chained) */

      g_sdmmc_dmadd[i].des2 = (uint32_t)priv->buffer +
                              (i * MCI_DMADES1_MAXTR);

      /* Setup basic control */

      ctrl = MCI_DMADES0_OWN | MCI_DMADES0_CH;

      if (i == 0)
        {
          ctrl |= MCI_DMADES0_FS; /* First DMA buffer */
        }

      /* No more data?  Then this is the last descriptor */

      if (buflen == 0)
        {
          ctrl |= MCI_DMADES0_LD;
          g_sdmmc_dmadd[i].des3 = 0;
        }
      else
        {
          ctrl |= MCI_DMADES0_DIC;
          g_sdmmc_dmadd[i].des3 = (uint32_t)&g_sdmmc_dmadd[i + 1];
        }

      g_sdmmc_dmadd[i].des0 = ctrl;
      i++;
    }

  DEBUGASSERT(i < NUM_DMA_DESCRIPTORS);

  lpc43_putreg((uint32_t)&g_sdmmc_dmadd[0], LPC43_SDMMC_DBADDR);

  /* Flush ints before we start */

  lpc43_putreg(SDCARD_TRANSFER_ALL, LPC43_SDMMC_RINTSTS);

  /* Enable internal DMA, burst size of 4, fixed burst */

  regval  = lpc43_getreg(LPC43_SDMMC_CTRL);
  regval |= SDMMC_CTRL_INTDMA;
  lpc43_putreg(regval, LPC43_SDMMC_CTRL);

  regval = SDMMC_BMOD_DE | SDMMC_BMOD_PBL_4XFRS;
  lpc43_putreg(regval, LPC43_SDMMC_BMOD);

  /* Setup DMA error interrupts */

  lpc43_config_dmaints(priv, SDCARD_DMARECV_MASK, SDCARD_DMAERROR_MASK);
  return OK;
}
#endif

/****************************************************************************
 * Name: lpc43_dmasendsetup
 *
 * Description:
 *   Setup to perform a write DMA.  If the processor supports a data cache,
 *   then this method will also make sure that the contents of the DMA memory
 *   and the data cache are coherent.  For write transfers, this may mean
 *   flushing the data cache.
 *
 * Input Parameters:
 *   dev    - An instance of the SD card device interface
 *   buffer - The memory to DMA into
 *   buflen - The size of the DMA transfer in bytes
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_SDMMC_DMA
static int lpc43_dmasendsetup(struct sdio_dev_s *dev,
                              const uint8_t *buffer, size_t buflen)
{
  struct lpc43_dev_s *priv = (struct lpc43_dev_s *)dev;
  uint32_t regval;
  uint32_t ctrl;
  uint32_t maxs;
  int i;

  /* Don't bother with DMA if the entire transfer will fit in the TX FIFO or
   * if we do not have a 4-bit wide bus.
   */

  DEBUGASSERT(priv != NULL);

  if (buflen <= LPC43_TXFIFO_SIZE || !priv->widebus)
    {
      return lpc43_sendsetup(dev, buffer, buflen);
    }

  mcinfo("buflen=%lu\n", (unsigned long)buflen);
  DEBUGASSERT(buffer != NULL && buflen > 0 && ((uint32_t)buffer & 3) == 0);

  /* Reset DMA controller internal registers.  The SWR bit automatically
   * clears in one clock cycle.
   */

  lpc43_putreg(SDMMC_BMOD_SWR, LPC43_SDMMC_BMOD);

  /* Save the destination buffer information for use by the interrupt
   * handler.
   */

  priv->buffer    = (uint32_t *)buffer;
  priv->remaining = buflen;
  priv->wrdir     = true;
  priv->dmamode   = true;

  /* Reset the FIFO and DMA */

  regval  = lpc43_getreg(LPC43_SDMMC_CTRL);
  regval |= SDMMC_CTRL_FIFORESET | SDMMC_CTRL_DMARESET;
  lpc43_putreg(regval, LPC43_SDMMC_CTRL);

  while ((lpc43_getreg(LPC43_SDMMC_CTRL) & (SDMMC_CTRL_FIFORESET |
                                            SDMMC_CTRL_DMARESET)) != 0)
    {
    }

  /* Configure the FIFO so that we will receive the DMA/FIFO requests
   * whenever there are FIFO_DEPTH/2 or fewer words in the FIFO.
   */

  regval = SDMMC_FIFOTH_TXWMARK(LPC43_TXFIFO_DEPTH / 2) |
           SDMMC_FIFOTH_DMABURST_4XFRS;
  lpc43_putreg(regval, LPC43_SDMMC_FIFOTH);

  /* Setup DMA descriptor list */

  i = 0;
  while (buflen > 0)
    {
      /* Limit size of the transfer to maximum buffer size */

      maxs = buflen;

      if (maxs > MCI_DMADES1_MAXTR)
        {
          maxs = MCI_DMADES1_MAXTR;
        }

      buflen -= maxs;

      /* Set buffer size */

      g_sdmmc_dmadd[i].des1 = MCI_DMADES1_BS1(maxs);

      /* Setup buffer address (chained) */

      g_sdmmc_dmadd[i].des2 = (uint32_t)priv->buffer +
                              (i * MCI_DMADES1_MAXTR);

      /* Setup basic control */

      ctrl = MCI_DMADES0_OWN | MCI_DMADES0_CH;

      if (i == 0)
        {
          ctrl |= MCI_DMADES0_FS; /* First DMA buffer */
        }

      /* No more data?  Then this is the last descriptor */

      if (buflen == 0)
        {
          ctrl |= MCI_DMADES0_LD;
          g_sdmmc_dmadd[i].des3 = 0;
        }
      else
        {
          ctrl |= MCI_DMADES0_DIC;
          g_sdmmc_dmadd[i].des3 = (uint32_t)&g_sdmmc_dmadd[i + 1];
        }

      g_sdmmc_dmadd[i].des0 = ctrl;
      i++;
    }

  DEBUGASSERT(i < NUM_DMA_DESCRIPTORS);

  lpc43_putreg((uint32_t) &g_sdmmc_dmadd[0], LPC43_SDMMC_DBADDR);

  /* Flush ints before we start */

  lpc43_putreg(SDCARD_TRANSFER_ALL, LPC43_SDMMC_RINTSTS);

  /* Enable internal DMA, burst size of 4, fixed burst */

  regval  = lpc43_getreg(LPC43_SDMMC_CTRL);
  regval |= SDMMC_CTRL_INTDMA;
  lpc43_putreg(regval, LPC43_SDMMC_CTRL);

  regval = SDMMC_BMOD_DE | SDMMC_BMOD_PBL_4XFRS;
  lpc43_putreg(regval, LPC43_SDMMC_BMOD);

  /* Setup DMA error interrupts */

  lpc43_config_dmaints(priv, SDCARD_DMASEND_MASK, SDCARD_DMAERROR_MASK);
  return OK;
}
#endif

/****************************************************************************
 * Name: lpc43_callback
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

static void lpc43_callback(struct lpc43_dev_s *priv)
{
  /* Is a callback registered? */

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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_sdmmc_initialize
 *
 * Description:
 *   Initialize the SD/MMC peripheral for normal operation.
 *
 * Input Parameters:
 *   slotno - Not used.
 *
 * Returned Value:
 *   A reference to an SD card interface structure.  NULL is returned on
 *   failures.
 *
 ****************************************************************************/

struct sdio_dev_s *lpc43_sdmmc_initialize(int slotno)
{
  struct lpc43_dev_s *priv = &g_scard_dev;
  irqstate_t flags;
  uint32_t regval;

  mcinfo("slotno=%d\n", slotno);
  flags = enter_critical_section();

  /* Set up the clock source */

  regval  = getreg32(LPC43_BASE_SDIO_CLK);
  regval &= ~BASE_SDIO_CLK_CLKSEL_MASK;
  regval |= (BOARD_SDIO_CLKSRC | BASE_SDIO_CLK_AUTOBLOCK);
  putreg32(regval, LPC43_BASE_SDIO_CLK);

  /* Enable clocking to the SD/MMC peripheral */

  regval  = lpc43_getreg(LPC43_CCU1_M4_SDIO_CFG);
  regval |= CCU_CLK_CFG_RUN;
  regval |= CCU_CLK_CFG_AUTO;
  regval |= CCU_CLK_CFG_WAKEUP;
  lpc43_putreg(regval, LPC43_CCU1_M4_SDIO_CFG);

  /* Setup the delay register */

  lpc43_putreg(LPC43_SDMMC_DELAY_DEFAULT, LPC43_SDMMC_DELAY);

  /* Initialize semaphores */

  nxsem_init(&priv->waitsem, 0, 0);

  /* The waitsem semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_set_protocol(&priv->waitsem, SEM_PRIO_NONE);

  /* Configure GPIOs for 4-bit, wide-bus operation */

  lpc43_pin_config(GPIO_SD_D0);
#ifndef CONFIG_SDIO_WIDTH_D1_ONLY
  lpc43_pin_config(GPIO_SD_D1);
  lpc43_pin_config(GPIO_SD_D2);
  lpc43_pin_config(GPIO_SD_D3);
#endif
#ifdef CONFIG_MMCSD_HAVE_CARDDETECT
  lpc43_pin_config(GPIO_SD_CARD_DET_N);
#endif
  lpc43_pin_config(GPIO_SD_CLK);
  lpc43_pin_config(GPIO_SD_CMD);
#ifdef CONFIG_LPC43_SDMMC_PWRCTRL
  lpc43_pin_config(GPIO_SD_POW_EN);
#endif
#ifdef CONFIG_MMCSD_HAVE_WRITEPROTECT
  lpc43_pin_config(GPIO_SD_WR_PRT);
#endif

  regval  = getreg32(LPC43_SCU_SFSCLK2);
  regval |= (2 << 3);   /* Disable pull-down and pull-up resistor */
  regval |= (1 << 6);   /* Enable Input buffer */
  regval |= (4);        /* Selects pin function 4 */
  putreg32(regval, LPC43_SCU_SFSCLK2);

  /* Reset the card and assure that it is in the initial, unconfigured
   * state.
   */

  lpc43_reset(&priv->dev);

  leave_critical_section(flags);
  return &g_scard_dev.dev;
}

#endif /* CONFIG_LPC43_SDMMC */
