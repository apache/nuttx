/****************************************************************************
 * arch/arm/src/sam34/sam_spi.c
 *
 *   Copyright (C) 2011, 2013-2014, 2016-2017 Gregory Nutt. All rights
 *     reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Diego Sanchez <dsanchez@nx-engineering.com>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "sam_gpio.h"
#include "sam_dmac.h"
#include "sam_cmcc.h"
#include "sam_periphclks.h"
#include "sam_spi.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_dmac.h"
#include "hardware/sam_spi.h"
#include "hardware/sam_pinmap.h"

#if defined(CONFIG_SAM34_SPI0) || defined(CONFIG_SAM34_SPI1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* When SPI DMA is enabled, small DMA transfers will still be performed by
 * polling logic.  But we need a threshold value to determine what is small.
 * That value is provided by CONFIG_SAM34_SPI_DMATHRESHOLD.
 */

#ifndef CONFIG_SAM34_SPI_DMATHRESHOLD
#  define CONFIG_SAM34_SPI_DMATHRESHOLD 4
#endif

#ifdef CONFIG_SAM34_SPI_DMA

#  if defined(CONFIG_SAM34_SPI0) && defined(CONFIG_SAM34_DMAC0)
#    define SAM34_SPI0_DMA true
#  else
#    define SAM34_SPI0_DMA false
#  endif

#  if defined(CONFIG_SAM34_SPI1) && defined(CONFIG_SAM34_DMAC1)
#    define SAM34_SPI1_DMA true
#  else
#    define SAM34_SPI1_DMA false
#  endif
#endif

#ifndef CONFIG_DEBUG_SPI_INFO
#  undef CONFIG_SAM34_SPI_REGDEBUG
#endif

#ifndef CONFIG_DEBUG_DMA_INFO
#  undef CONFIG_SAM34_SPI_DMADEBUG
#endif

/* Clocking *****************************************************************/

/* Select MCU-specific settings
 *
 * For the SAM3U, SAM3A, SAM3X, SAM4E and SAM4S SPI is driven by the main
 *   clock.
 * For the SAM4L, SPI is driven by CLK_SPI which is the PBB clock.
 */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A) || defined(CONFIG_ARCH_CHIP_SAM4S) || \
    defined(CONFIG_ARCH_CHIP_SAM4E)
#  define SAM_SPI_CLOCK  BOARD_MCK_FREQUENCY  /* Frequency of the main clock */
#elif defined(CONFIG_ARCH_CHIP_SAM4L)
#  define SAM_SPI_CLOCK  BOARD_PBB_FREQUENCY  /* PBB frequency */
#else
#  error Unrecognized SAM architecture
#endif

/* DMA timeout.  The value is not critical; we just don't want the system to
 * hang in the event that a DMA does not finish.  This is set to
 */

#define DMA_TIMEOUT_MS    (800)
#define DMA_TIMEOUT_TICKS MSEC2TICK(DMA_TIMEOUT_MS)

/* Debug ********************************************************************/

/* Check if SPI debut is enabled */

#ifndef CONFIG_DEBUG_DMA_INFO
#  undef CONFIG_SAM34_SPI_DMADEBUG
#endif

#define DMA_INITIAL      0
#define DMA_AFTER_SETUP  1
#define DMA_AFTER_START  2
#define DMA_CALLBACK     3
#define DMA_TIMEOUT      3
#define DMA_END_TRANSFER 4
#define DMA_NSAMPLES     5

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of the one SPI chip select */

struct sam_spics_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */

  uint32_t frequency;          /* Requested clock frequency */
  uint32_t actual;             /* Actual clock frequency */
  uint8_t mode;                /* Mode 0,1,2,3 */
  uint8_t nbits;               /* Width of word in bits (8 to 16) */

#if defined(CONFIG_SAM34_SPI0) || defined(CONFIG_SAM34_SPI1)
  uint8_t spino;               /* SPI controller number (0 or 1) */
#endif
  uint8_t cs;                  /* Chip select number */

#ifdef CONFIG_SAM34_SPI_DMA
  bool candma;                 /* DMA is supported */
  sem_t dmawait;               /* Used to wait for DMA completion */
  struct wdog_s dmadog;        /* Watchdog that handles DMA timeouts */
  int result;                  /* DMA result */
  DMA_HANDLE rxdma;            /* SPI RX DMA handle */
  DMA_HANDLE txdma;            /* SPI TX DMA handle */
#endif

  /* Debug stuff */

#ifdef CONFIG_SAM34_SPI_DMADEBUG
  struct sam_dmaregs_s rxdmaregs[DMA_NSAMPLES];
  struct sam_dmaregs_s txdmaregs[DMA_NSAMPLES];
#endif
};

/* Type of board-specific SPI status function */

typedef void (*select_t)(uint32_t devid, bool selected);

/* Chip select register offsets */

/* The overall state of one SPI controller */

struct sam_spidev_s
{
  uint32_t base;               /* SPI controller register base address */
  sem_t spisem;                /* Assures mutually exclusive access to SPI */
  select_t select;             /* SPI select call-out */
  bool initialized;            /* TRUE: Controller has been initialized */
#ifdef CONFIG_SAM34_SPI_DMA
  uint8_t rxintf;              /* RX hardware interface number */
  uint8_t txintf;              /* TX hardware interface number */
#endif

  /* Debug stuff */

#ifdef CONFIG_SAM34_SPI_REGDEBUG
  bool     wrlast;            /* Last was a write */
  uint32_t addresslast;       /* Last address */
  uint32_t valuelast;         /* Last value */
  int      ntimes;            /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

#ifdef CONFIG_SAM34_SPI_REGDEBUG
static bool     spi_checkreg(struct sam_spidev_s *spi, bool wr,
                  uint32_t value, uint32_t address);
#else
# define        spi_checkreg(spi,wr,value,address) (false)
#endif

static inline uint32_t spi_getreg(struct sam_spidev_s *spi,
                  unsigned int offset);
static inline void spi_putreg(struct sam_spidev_s *spi, uint32_t value,
                  unsigned int offset);
static inline struct sam_spidev_s *spi_device(struct sam_spics_s *spics);

#ifdef CONFIG_DEBUG_SPI_INFO
static void     spi_dumpregs(struct sam_spidev_s *spi, const char *msg);
#else
# define        spi_dumpregs(spi,msg)
#endif

static inline void spi_flush(struct sam_spidev_s *spi);
static inline uint32_t spi_cs2pcs(struct sam_spics_s *spics);

/* DMA support */

#ifdef CONFIG_SAM34_SPI_DMA

#ifdef CONFIG_SAM34_SPI_DMADEBUG
#  define spi_rxdma_sample(s,i) sam_dmasample((s)->rxdma, &(s)->rxdmaregs[i])
#  define spi_txdma_sample(s,i) sam_dmasample((s)->txdma, &(s)->txdmaregs[i])
static void     spi_dma_sampleinit(struct sam_spics_s *spics);
static void     spi_dma_sampledone(struct sam_spics_s *spics);

#else
#  define spi_rxdma_sample(s,i)
#  define spi_txdma_sample(s,i)
#  define spi_dma_sampleinit(s)
#  define spi_dma_sampledone(s)

#endif

static void     spi_rxcallback(DMA_HANDLE handle, void *arg, int result);
static void     spi_txcallback(DMA_HANDLE handle, void *arg, int result);
static inline uintptr_t spi_regaddr(struct sam_spics_s *spics,
                  unsigned int offset);
#endif

/* SPI methods */

static int      spi_lock(struct spi_dev_s *dev, bool lock);
static void     spi_select(struct spi_dev_s *dev, uint32_t devid,
                  bool selected);
static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency);
static void     spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SAM34_SPI_DMA
static void     spi_exchange_nodma(struct spi_dev_s *dev,
                   const void *txbuffer, void *rxbuffer, size_t nwords);
#endif
static void     spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                   void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void     spi_sndblock(struct spi_dev_s *dev,
                   const void *buffer, size_t nwords);
static void     spi_recvblock(struct spi_dev_s *dev, void *buffer,
                   size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array maps chip select numbers (0-3) to CSR register offsets */

static const uint8_t g_csroffset[4] =
{
  SAM_SPI_CSR0_OFFSET, SAM_SPI_CSR1_OFFSET,
  SAM_SPI_CSR2_OFFSET, SAM_SPI_CSR3_OFFSET
};

#ifdef CONFIG_SAM34_SPI0
/* SPI0 driver operations */

static const struct spi_ops_s g_spi0ops =
{
  .lock              = spi_lock,
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                 /* Not supported */
#endif
  .status            = sam_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = sam_spi0cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

/* This is the overall state of the SPI0 controller */

static struct sam_spidev_s g_spi0dev =
{
  .base              = SAM_SPI0_BASE,
  .select            = sam_spi0select,
#ifdef CONFIG_SAM34_SPI_DMA
  .rxintf            = DMACHAN_INTF_SPI0RX,
  .txintf            = DMACHAN_INTF_SPI0TX,
#endif
};
#endif

#ifdef CONFIG_SAM34_SPI1
/* SPI1 driver operations */

static const struct spi_ops_s g_spi1ops =
{
  .lock              = spi_lock,
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = sam_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = sam_spi1cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

/* This is the overall state of the SPI0 controller */

static struct sam_spidev_s g_spi1dev =
{
  .base              = SAM_SPI1_BASE,
  .select            = sam_spi1select,
#ifdef CONFIG_SAM34_SPI_DMA
  .rxintf            = DMACHAN_INTF_SPI1RX,
  .txintf            = DMACHAN_INTF_SPI1TX,
#endif
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_checkreg
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

#ifdef CONFIG_SAM34_SPI_REGDEBUG
static bool spi_checkreg(struct sam_spidev_s *spi, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr      == spi->wrlast &&     /* Same kind of access? */
      value   == spi->valuelast &&  /* Same value? */
      address == spi->addresslast)  /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      spi->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (spi->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          spiinfo("...[Repeats %d times]...\n", spi->ntimes);
        }

      /* Save information about the new access */

      spi->wrlast      = wr;
      spi->valuelast   = value;
      spi->addresslast = address;
      spi->ntimes      = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *  Read an SPI register
 *
 ****************************************************************************/

static inline uint32_t spi_getreg(struct sam_spidev_s *spi,
                                  unsigned int offset)
{
  uint32_t address = spi->base + offset;
  uint32_t value = getreg32(address);

#ifdef CONFIG_SAM34_SPI_REGDEBUG
  if (spi_checkreg(spi, false, value, address))
    {
      spiinfo("%08x->%08x\n", address, value);
    }
#endif

  return value;
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *  Write a value to an SPI register
 *
 ****************************************************************************/

static inline void spi_putreg(struct sam_spidev_s *spi, uint32_t value,
                              unsigned int offset)
{
  uint32_t address = spi->base + offset;

#ifdef CONFIG_SAM34_SPI_REGDEBUG
  if (spi_checkreg(spi, true, value, address))
    {
      spiinfo("%08x<-%08x\n", address, value);
    }
#endif

  putreg32(value, address);
}

/****************************************************************************
 * Name: spi_dumpregs
 *
 * Description:
 *   Dump the contents of all SPI registers
 *
 * Input Parameters:
 *   spi - The SPI controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SPI_INFO
static void spi_dumpregs(struct sam_spidev_s *spi, const char *msg)
{
  spiinfo("%s:\n", msg);
  spiinfo("    MR:%08x   SR:%08x  IMR:%08x\n",
          getreg32(spi->base + SAM_SPI_MR_OFFSET),
          getreg32(spi->base + SAM_SPI_SR_OFFSET),
          getreg32(spi->base + SAM_SPI_IMR_OFFSET));
  spiinfo("  CSR0:%08x CSR1:%08x CSR2:%08x CSR3:%08x\n",
          getreg32(spi->base + SAM_SPI_CSR0_OFFSET),
          getreg32(spi->base + SAM_SPI_CSR1_OFFSET),
          getreg32(spi->base + SAM_SPI_CSR2_OFFSET),
          getreg32(spi->base + SAM_SPI_CSR3_OFFSET));
  spiinfo("  WPCR:%08x WPSR:%08x\n",
          getreg32(spi->base + SAM_SPI_WPCR_OFFSET),
          getreg32(spi->base + SAM_SPI_WPSR_OFFSET));
}
#endif

/****************************************************************************
 * Name: spi_device
 *
 * Description:
 *    Given a chip select instance, return a pointer to the parent SPI
 *    controller instance.
 *
 ****************************************************************************/

static inline struct sam_spidev_s *spi_device(struct sam_spics_s *spics)
{
#if defined(CONFIG_SAM34_SPI0) && defined(CONFIG_SAM34_SPI1)
  return spics->spino ? &g_spi1dev : &g_spi0dev;
#elif defined(CONFIG_SAM34_SPI0)
  return &g_spi0dev;
#else
  return &g_spi1dev;
#endif
}

/****************************************************************************
 * Name: spi_flush
 *
 * Description:
 *   Make sure that there are now dangling SPI transfer in progress
 *
 * Input Parameters:
 *   spi - SPI controller state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_flush(struct sam_spidev_s *spi)
{
  /* Make sure the no TX activity is in progress... waiting if necessary */

  while ((spi_getreg(spi, SAM_SPI_SR_OFFSET) & SPI_INT_TXEMPTY) == 0);

  /* Then make sure that there is no pending RX data .. reading as
   * discarding as necessary.
   */

  while ((spi_getreg(spi, SAM_SPI_SR_OFFSET) & SPI_INT_RDRF) != 0)
    {
       spi_getreg(spi, SAM_SPI_RDR_OFFSET);
    }
}

/****************************************************************************
 * Name: spi_cs2pcs
 *
 * Description:
 *   Map the chip select number to the bit-set PCS field used in the SPI
 *   registers.  A chip select number is used for indexing and identifying
 *   chip selects.  However, the chip select information is represented by
 *   a bit set in the SPI registers.  This function maps those chip select
 *   numbers to the correct bit set:
 *
 *    CS  Returned   Spec    Effective
 *    No.   PCS      Value    NPCS
 *   ---- --------  -------- --------
 *    0    0000      xxx0     1110
 *    1    0001      xx01     1101
 *    2    0011      x011     1011
 *    3    0111      0111     0111
 *
 * Input Parameters:
 *   spics - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t spi_cs2pcs(struct sam_spics_s *spics)
{
  return ((uint32_t)1 << (spics->cs)) - 1;
}

/****************************************************************************
 * Name: spi_dma_sampleinit
 *
 * Description:
 *   Initialize sampling of DMA registers (if CONFIG_SAM34_SPI_DMADEBUG)
 *
 * Input Parameters:
 *   spics - Chip select doing the DMA
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_SPI_DMADEBUG
static void spi_dma_sampleinit(struct sam_spics_s *spics)
{
  /* Put contents of register samples into a known state */

  memset(spics->rxdmaregs, 0xff,
         DMA_NSAMPLES * sizeof(struct sam_dmaregs_s));
  memset(spics->txdmaregs, 0xff,
         DMA_NSAMPLES * sizeof(struct sam_dmaregs_s));

  /* Then get the initial samples */

  sam_dmasample(spics->rxdma, &spics->rxdmaregs[DMA_INITIAL]);
  sam_dmasample(spics->txdma, &spics->txdmaregs[DMA_INITIAL]);
}
#endif

/****************************************************************************
 * Name: spi_dma_sampledone
 *
 * Description:
 *   Dump sampled DMA registers
 *
 * Input Parameters:
 *   spics - Chip select doing the DMA
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_SPI_DMADEBUG
static void spi_dma_sampledone(struct sam_spics_s *spics)
{
  /* Sample the final registers */

  sam_dmasample(spics->rxdma, &spics->rxdmaregs[DMA_END_TRANSFER]);
  sam_dmasample(spics->txdma, &spics->txdmaregs[DMA_END_TRANSFER]);

  /* Then dump the sampled DMA registers */

  /* Initial register values */

  sam_dmadump(spics->txdma, &spics->txdmaregs[DMA_INITIAL],
              "TX: Initial Registers");
  sam_dmadump(spics->rxdma, &spics->rxdmaregs[DMA_INITIAL],
              "RX: Initial Registers");

  /* Register values after DMA setup */

  sam_dmadump(spics->txdma, &spics->txdmaregs[DMA_AFTER_SETUP],
              "TX: After DMA Setup");
  sam_dmadump(spics->rxdma, &spics->rxdmaregs[DMA_AFTER_SETUP],
              "RX: After DMA Setup");

  /* Register values after DMA start */

  sam_dmadump(spics->txdma, &spics->txdmaregs[DMA_AFTER_START],
              "TX: After DMA Start");
  sam_dmadump(spics->rxdma, &spics->rxdmaregs[DMA_AFTER_START],
              "RX: After DMA Start");

  /* Register values at the time of the TX and RX DMA callbacks
   * -OR- DMA timeout.
   *
   * If the DMA timed out, then there will not be any RX DMA
   * callback samples.  There is probably no TX DMA callback
   * samples either, but we don't know for sure.
   */

  sam_dmadump(spics->txdma, &spics->txdmaregs[DMA_CALLBACK],
              "TX: At DMA callback");

  /* Register values at the end of the DMA */

  if (spics->result == -ETIMEDOUT)
    {
      sam_dmadump(spics->rxdma, &spics->rxdmaregs[DMA_TIMEOUT],
                  "RX: At DMA timeout");
    }
  else
    {
      sam_dmadump(spics->rxdma, &spics->rxdmaregs[DMA_CALLBACK],
                  "RX: At DMA callback");
    }

  sam_dmadump(spics->txdma, &spics->txdmaregs[DMA_END_TRANSFER],
              "TX: At End-of-Transfer");
  sam_dmadump(spics->rxdma, &spics->rxdmaregs[DMA_END_TRANSFER],
              "RX: At End-of-Transfer");
}
#endif

/****************************************************************************
 * Name: spi_dmatimeout
 *
 * Description:
 *   The watchdog timeout setup when a has expired without completion of a
 *   DMA.
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

#ifdef CONFIG_SAM34_SPI_DMA
static void spi_dmatimeout(wdparm_t arg)
{
  struct sam_spics_s *spics = (struct sam_spics_s *)arg;
  DEBUGASSERT(spics != NULL);

  /* Sample DMA registers at the time of the timeout */

  spi_rxdma_sample(spics, DMA_CALLBACK);

  /* Report timeout result, perhaps overwriting any failure reports from
   * the TX callback.
   */

  spics->result = -ETIMEDOUT;

  /* Then wake up the waiting thread */

  nxsem_post(&spics->dmawait);
}
#endif

/****************************************************************************
 * Name: spi_rxcallback
 *
 * Description:
 *   This callback function is invoked at the completion of the SPI RX DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   arg - A pointer to the chip select structure
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_SPI_DMA
static void spi_rxcallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_spics_s *spics = (struct sam_spics_s *)arg;
  DEBUGASSERT(spics != NULL);

  /* Cancel the watchdog timeout */

  wd_cancel(&spics->dmadog);

  /* Sample DMA registers at the time of the callback */

  spi_rxdma_sample(spics, DMA_CALLBACK);

  /* Report the result of the transfer only if the TX callback has not
   * already reported an error.
   */

  if (spics->result == -EBUSY)
    {
      /* Save the result of the transfer if no error was previously
       * reported
       */

      spics->result = result;
    }

  /* Then wake up the waiting thread */

  nxsem_post(&spics->dmawait);
}
#endif

/****************************************************************************
 * Name: spi_txcallback
 *
 * Description:
 *   This callback function is invoked at the completion of the SPI TX DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   arg - A pointer to the chip select structure
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_SPI_DMA
static void spi_txcallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_spics_s *spics = (struct sam_spics_s *)arg;
  DEBUGASSERT(spics != NULL);

  spi_txdma_sample(spics, DMA_CALLBACK);

  /* Do nothing on the TX callback unless an error is reported.  This
   * callback is not really important because the SPI exchange is not
   * complete until the RX callback is received.
   */

  if (result != OK && spics->result == -EBUSY)
    {
      /* Save the result of the transfer if an error is reported */

      spics->result = result;
    }
}
#endif

/****************************************************************************
 * Name: spi_regaddr
 *
 * Description:
 *   Return the address of an SPI register
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_SPI_DMA
static inline uintptr_t spi_regaddr(struct sam_spics_s *spics,
                                    unsigned int offset)
{
  struct sam_spidev_s *spi = spi_device(spics);
  return spi->base + offset;
}
#endif

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct sam_spics_s *spics = (struct sam_spics_s *)dev;
  struct sam_spidev_s *spi = spi_device(spics);
  int ret;

  spiinfo("lock=%d\n", lock);
  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&spi->spisem);
    }
  else
    {
      ret = nxsem_post(&spi->spisem);
    }

  return ret;
}

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   This function does not actually set the chip select line.  Rather, it
 *   simply maps the device ID into a chip select number and retains that
 *   chip select number for later use.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static void spi_select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  struct sam_spics_s *spics = (struct sam_spics_s *)dev;
  struct sam_spidev_s *spi = spi_device(spics);
  uint32_t regval;

  /* Are we selecting or de-selecting the device? */

  spiinfo("selected=%d\n", selected);
  if (selected)
    {
      spiinfo("cs=%d\n", spics->cs);

      /* Before writing the TDR, the PCS field in the SPI_MR register must
       * be set in order to select a slave.
       */

      regval  = spi_getreg(spi, SAM_SPI_MR_OFFSET);
      regval &= ~SPI_MR_PCS_MASK;
      regval |= (spi_cs2pcs(spics) << SPI_MR_PCS_SHIFT);
      spi_putreg(spi, regval, SAM_SPI_MR_OFFSET);
    }

  /* Perform any board-specific chip select operations. PIO chip select
   * pins may be programmed by the board specific logic in one of two
   * different ways.  First, the pins may be programmed as SPI peripherals.
   * In that case, the pins are completely controlled by the SPI driver.
   * The sam_spi[0|1]select methods still needs to be provided, but they
   * may be only stubs.
   *
   * An alternative way to program the PIO chip select pins is as normal
   * PIO outputs.  In that case, the automatic control of the CS pins is
   * bypassed and this function must provide control of the chip select.
   * NOTE:  In this case, the PIO output pin does *not* have to be the
   * same as the NPCS pin normal associated with the chip select number.
   */

  spi->select(devid, selected);
}

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
  struct sam_spics_s *spics = (struct sam_spics_s *)dev;
  struct sam_spidev_s *spi = spi_device(spics);
  uint32_t actual;
  uint32_t scbr;
  uint32_t dlybs;
  uint32_t dlybct;
  uint32_t regval;
  unsigned int offset;

  spiinfo("cs=%d frequency=%d\n", spics->cs, frequency);

  /* Check if the requested frequency is the same as the frequency
   * selection
   */

  if (spics->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return spics->actual;
    }

  /* Configure SPI to a frequency as close as possible to the requested
   * frequency.
   *
   *   SPCK frequency = SPI_CLK / SCBR, or SCBR = SPI_CLK / frequency
   */

  scbr = SAM_SPI_CLOCK / frequency;

  if (scbr < 8)
    {
      scbr = 8;
    }
  else if (scbr > 254)
    {
      scbr = 254;
    }

  scbr = (scbr + 1) & ~1;

  /* Save the new scbr value */

  offset = (unsigned int)g_csroffset[spics->cs];
  regval  = spi_getreg(spi, offset);
  regval &= ~(SPI_CSR_SCBR_MASK | SPI_CSR_DLYBS_MASK | SPI_CSR_DLYBCT_MASK);
  regval |= scbr << SPI_CSR_SCBR_SHIFT;

  /* DLYBS: Delay Before SPCK.  This field defines the delay from NPCS valid
   * to the first valid SPCK transition. When DLYBS equals zero, the NPCS
   * valid to SPCK transition is 1/2 the SPCK clock period. Otherwise, the
   * following equations determine the delay:
   *
   *   Delay Before SPCK = DLYBS / SPI_CLK
   *
   * For a 2uS delay
   *
   *   DLYBS = SPI_CLK * 0.000002 = SPI_CLK / 500000
   */

  dlybs   = SAM_SPI_CLOCK / 500000;
  regval |= dlybs << SPI_CSR_DLYBS_SHIFT;

  /* DLYBCT: Delay Between Consecutive Transfers.  This field defines the
   * delay between two consecutive transfers with the same peripheral without
   * removing the chip select. The delay is always inserted after each
   * transfer and before removing the chip select if needed.
   *
   *  Delay Between Consecutive Transfers = (32 x DLYBCT) / SPI_CLK
   *
   * For a 5uS delay:
   *
   *  DLYBCT = SPI_CLK * 0.000005 / 32 = SPI_CLK / 200000 / 32
   */

  dlybct  = SAM_SPI_CLOCK / 200000 / 32;
  regval |= dlybct << SPI_CSR_DLYBCT_SHIFT;
  spi_putreg(spi, regval, offset);

  /* Calculate the new actual frequency */

  actual = SAM_SPI_CLOCK / scbr;
  spiinfo("csr[offset=%02x]=%08x actual=%d\n", offset, regval, actual);

  /* Save the frequency setting */

  spics->frequency = frequency;
  spics->actual    = actual;

  spiinfo("Frequency %d->%d\n", frequency, actual);
  return actual;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode. Optional.  See enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct sam_spics_s *spics = (struct sam_spics_s *)dev;
  struct sam_spidev_s *spi = spi_device(spics);
  uint32_t regval;
  unsigned int offset;

  spiinfo("cs=%d mode=%d\n", spics->cs, mode);

  /* Has the mode changed? */

  if (mode != spics->mode)
    {
      /* Yes... Set the mode appropriately:
       *
       * SPI  CPOL NCPHA
       * MODE
       *  0    0    1
       *  1    0    0
       *  2    1    1
       *  3    1    0
       */

      offset  = (unsigned int)g_csroffset[spics->cs];
      regval  = spi_getreg(spi, offset);
      regval &= ~(SPI_CSR_CPOL | SPI_CSR_NCPHA);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; NCPHA=1 */
          regval |= SPI_CSR_NCPHA;
          break;

        case SPIDEV_MODE1: /* CPOL=0; NCPHA=0 */
          break;

        case SPIDEV_MODE2: /* CPOL=1; NCPHA=1 */
          regval |= (SPI_CSR_CPOL | SPI_CSR_NCPHA);
          break;

        case SPIDEV_MODE3: /* CPOL=1; NCPHA=0 */
          regval |= SPI_CSR_CPOL;
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

      spi_putreg(spi, regval, offset);
      spiinfo("csr[offset=%02x]=%08x\n", offset, regval);

      /* Save the mode so that subsequent re-configurations will be faster */

      spics->mode = mode;
    }
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct sam_spics_s *spics = (struct sam_spics_s *)dev;
  struct sam_spidev_s *spi = spi_device(spics);
  uint32_t regval;
  unsigned int offset;

  spiinfo("cs=%d nbits=%d\n", spics->cs, nbits);
  DEBUGASSERT(spics && nbits > 7 && nbits < 17);

  /* Has the number of bits changed? */

  if (nbits != spics->nbits)
    {
      /* Yes... Set number of bits appropriately */

      offset  = (unsigned int)g_csroffset[spics->cs];
      regval  = spi_getreg(spi, offset);
      regval &= ~SPI_CSR_BITS_MASK;
      regval |= SPI_CSR_BITS(nbits);
      spi_putreg(spi, regval, offset);

      spiinfo("csr[offset=%02x]=%08x\n", offset, regval);

      /* Save the selection so the subsequence re-configurations will be
       * faster.
       */

      spics->nbits = nbits;
    }
}

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  uint8_t txbyte;
  uint8_t rxbyte;

  /* spi_exchange can do this. Note: right now, this only deals with 8-bit
   * words.  If the SPI interface were configured for words of other sizes,
   * this would fail.
   */

  txbyte = (uint8_t)wd;
  rxbyte = (uint8_t)0;
  spi_exchange(dev, &txbyte, &rxbyte, 1);

  spiinfo("Sent %02x received %02x\n", txbyte, rxbyte);
  return (uint32_t)rxbyte;
}

/****************************************************************************
 * Name: spi_exchange (and spi_exchange_nodma)
 *
 * Description:
 *   Exchange a block of data from SPI.  There are two versions of this
 *   function:  (1) One that is enabled only when CONFIG_SAM34_SPI_DMA=y
 *   that performs DMA SPI transfers, but only when a larger block of
 *   data is being transferred.  And (2) another version that does polled
 *   SPI transfers.  When CONFIG_SAM34_SPI_DMA=n the latter is the only
 *   version available; when CONFIG_SAM34_SPI_DMA=y, this version is only
 *   used for short SPI transfers and gets renamed as spi_exchange_nodma).
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAM34_SPI_DMA
static void spi_exchange_nodma(struct spi_dev_s *dev, const void *txbuffer,
                               void *rxbuffer, size_t nwords)
#else
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
#endif
{
  struct sam_spics_s *spics = (struct sam_spics_s *)dev;
  struct sam_spidev_s *spi = spi_device(spics);
  uint32_t pcs;
  uint32_t data;
  uint16_t *rxptr16;
  uint16_t *txptr16;
  uint8_t *rxptr8;
  uint8_t *txptr8;

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n",
          txbuffer, rxbuffer, nwords);

  /* Set up PCS bits */

  pcs = spi_cs2pcs(spics) << SPI_TDR_PCS_SHIFT;

  /* Set up working pointers */

  if (spics->nbits > 8)
    {
      rxptr16 = (uint16_t *)rxbuffer;
      txptr16 = (uint16_t *)txbuffer;
      rxptr8  = NULL;
      txptr8  = NULL;
    }
  else
    {
      rxptr16 = NULL;
      txptr16 = NULL;
      rxptr8  = (uint8_t *)rxbuffer;
      txptr8  = (uint8_t *)txbuffer;
    }

  /* Make sure that any previous transfer is flushed from the hardware */

  spi_flush(spi);

  /* Loop, sending each word in the user-provided data buffer.
   *
   * Note 1: Good SPI performance would require that we implement DMA
   *         transfers!
   * Note 2: This loop might be made more efficient.  Would logic
   *         like the following improve the throughput?  Or would it
   *         just add the risk of overruns?
   *
   *   Get word 1;
   *   Send word 1;  Now word 1 is "in flight"
   *   nwords--;
   *   for ( ; nwords > 0; nwords--)
   *     {
   *       Get word N.
   *       Wait for TDRE meaning that word N-1 has moved to the shift
   *          register.
   *       Disable interrupts to keep the following atomic
   *       Send word N.  Now both work N-1 and N are "in flight"
   *       Wait for RDRF meaning that word N-1 is available
   *       Read word N-1.
   *       Re-enable interrupts.
   *       Save word N-1.
   *     }
   *   Wait for RDRF meaning that the final word is available
   *   Read the final word.
   *   Save the final word.
   */

  for (; nwords > 0; nwords--)
    {
      /* Get the data to send (0xff if there is no data source). */

      if (txptr8)
        {
          data = (uint32_t)*txptr8++;
        }
      else if (txptr16)
        {
          data = (uint32_t)*txptr16++;
        }
      else
        {
          data = 0xffff;
        }

      /* Set the PCS field in the value written to the TDR */

      data |= pcs;

      /* Do we need to set the LASTXFER bit in the TDR value too? */

#ifdef CONFIG_SPI_VARSELECT
      if (nwords == 1)
        {
          data |= SPI_TDR_LASTXFER;
        }
#endif

      /* Wait for any previous data written to the TDR to be transferred
       * to the serializer.
       */

      while ((spi_getreg(spi, SAM_SPI_SR_OFFSET) & SPI_INT_TDRE) == 0);

      /* Write the data to transmitted to the Transmit Data Register (TDR) */

      spi_putreg(spi, data, SAM_SPI_TDR_OFFSET);

      /* Wait for the read data to be available in the RDR.
       * TODO:  Data transfer rates would be improved using the RX FIFO
       *        (and also DMA)
       */

      while ((spi_getreg(spi, SAM_SPI_SR_OFFSET) & SPI_INT_RDRF) == 0);

      /* Read the received data from the SPI Data Register. */

      data = spi_getreg(spi, SAM_SPI_RDR_OFFSET);
      if (rxptr8)
        {
          *rxptr8++ = (uint8_t)data;
        }
      else if (rxptr16)
        {
          *rxptr16++ = (uint16_t)data;
        }
    }
}

#ifdef CONFIG_SAM34_SPI_DMA
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct sam_spics_s *spics = (struct sam_spics_s *)dev;
  struct sam_spidev_s *spi = spi_device(spics);
  uint32_t rxflags;
  uint32_t txflags;
  uint32_t txdummy;
  uint32_t rxdummy;
  uint32_t regaddr;
  uint32_t memaddr;
  uint32_t width;
  size_t nbytes;
  int ret;

  /* Convert the number of word to a number of bytes */

  nbytes = (spics->nbits > 8) ? nwords << 1 : nwords;

  /* If we cannot do DMA -OR- if this is a small SPI transfer, then let
   * spi_exchange_nodma() do the work.
   */

  if (!spics->candma || nbytes <= CONFIG_SAM34_SPI_DMATHRESHOLD)
    {
      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n",
          txbuffer, rxbuffer, nwords);

  spics = (struct sam_spics_s *)dev;
  spi = spi_device(spics);
  DEBUGASSERT(spics && spi);

  /* Make sure that any previous transfer is flushed from the hardware */

  spi_flush(spi);

  /* Sample initial DMA registers */

  spi_dma_sampleinit(spics);

  /* Select the source and destination width bits */

  if (spics->nbits > 8)
    {
      width = (DMACH_FLAG_PERIPHWIDTH_16BITS | DMACH_FLAG_MEMWIDTH_16BITS);
    }
  else
    {
      width = (DMACH_FLAG_PERIPHWIDTH_8BITS | DMACH_FLAG_MEMWIDTH_8BITS);
    }

  /* Configure the DMA channels.  There are four different cases:
   *
   * 1) A true exchange with the memory address incrementing on both
   *    RX and TX channels,
   * 2) A read operation with the memory address incrementing only on
   *    the receive channel,
   * 3) A write operation where the memory address increments only on
   *    the receive channel, and
   * 4) A corner case where there the memory address does not increment
   *    on either channel.  This case might be used in certain cases
   *    where you want to assure that certain number of clocks are
   *    provided on the SPI bus.
   */

  /* Configure the RX DMA channel */

  rxflags = DMACH_FLAG_FIFOCFG_LARGEST |
            ((uint32_t)spi->rxintf << DMACH_FLAG_PERIPHPID_SHIFT) |
            DMACH_FLAG_PERIPHH2SEL | DMACH_FLAG_PERIPHISPERIPH |
            DMACH_FLAG_PERIPHCHUNKSIZE_1 |
            ((uint32_t)(15) << DMACH_FLAG_MEMPID_SHIFT) |
            DMACH_FLAG_MEMCHUNKSIZE_1;

  /* Set the source and destination width bits */

  rxflags |= width;

  /* Handle the case where there is no sink buffer */

  if (!rxbuffer)
    {
      /* No sink data buffer.  Point to our dummy buffer and leave
       * the rxflags so that no address increment is performed.
       */

      rxbuffer = (void *)&rxdummy;
    }
  else
    {
      /* A receive buffer is available.
       *
       * Invalidate the RX buffer memory to force re-fetching from RAM when
       * the DMA completes
       */

      sam_cmcc_invalidate((uintptr_t)rxbuffer, (uintptr_t)rxbuffer + nbytes);

      /* Use normal RX memory incrementing. */

      rxflags |= DMACH_FLAG_MEMINCREMENT;
    }

  /* Configure the TX DMA channel */

  txflags = DMACH_FLAG_FIFOCFG_LARGEST |
            ((uint32_t)spi->txintf << DMACH_FLAG_PERIPHPID_SHIFT) |
            DMACH_FLAG_PERIPHH2SEL | DMACH_FLAG_PERIPHISPERIPH |
            DMACH_FLAG_PERIPHCHUNKSIZE_1 |
            ((uint32_t)(15) << DMACH_FLAG_MEMPID_SHIFT) |
            DMACH_FLAG_MEMCHUNKSIZE_1;

  /* Set the source and destination width bits */

  txflags |= width;

  /* Handle the case where there is no source buffer */

  if (!txbuffer)
    {
      /* No source data buffer.  Point to our dummy buffer and leave
       * the txflags so that no address increment is performed.
       */

      txdummy  = 0xffffffff;
      txbuffer = (const void *)&txdummy;
    }
  else
    {
      /* Source data is available.  Use normal TX memory incrementing. */

      txflags |= DMACH_FLAG_MEMINCREMENT;
    }

  /* Then configure the DMA channels to make it so */

  sam_dmaconfig(spics->rxdma, rxflags);
  sam_dmaconfig(spics->txdma, txflags);

  /* Configure the RX side of the exchange transfer */

  regaddr = spi_regaddr(spics, SAM_SPI_RDR_OFFSET);
  memaddr = (uintptr_t)rxbuffer;

  ret = sam_dmarxsetup(spics->rxdma, regaddr, memaddr, nwords);
  if (ret < 0)
    {
      dmaerr("ERROR: sam_dmarxsetup failed: %d\n", ret);
      return;
    }

  spi_rxdma_sample(spics, DMA_AFTER_SETUP);

  /* Configure the TX side of the exchange transfer */

  regaddr = spi_regaddr(spics, SAM_SPI_TDR_OFFSET);
  memaddr = (uintptr_t)txbuffer;

  ret = sam_dmatxsetup(spics->txdma, regaddr, memaddr, nwords);
  if (ret < 0)
    {
      dmaerr("ERROR: sam_dmatxsetup failed: %d\n", ret);
      return;
    }

  spi_txdma_sample(spics, DMA_AFTER_SETUP);

  /* Start the DMA transfer */

  spics->result = -EBUSY;
  ret = sam_dmastart(spics->rxdma, spi_rxcallback, (void *)spics);
  if (ret < 0)
    {
      dmaerr("ERROR: RX sam_dmastart failed: %d\n", ret);
      return;
    }

  spi_rxdma_sample(spics, DMA_AFTER_START);

  ret = sam_dmastart(spics->txdma, spi_txcallback, (void *)spics);
  if (ret < 0)
    {
      dmaerr("ERROR: RX sam_dmastart failed: %d\n", ret);
      sam_dmastop(spics->rxdma);
      return;
    }

  spi_txdma_sample(spics, DMA_AFTER_START);

  /* Wait for DMA completion.  This is done in a loop because there may be
   * false alarm semaphore counts that cause sam_wait() not fail to wait
   * or to wake-up prematurely (for example due to the receipt of a signal).
   * We know that the DMA has completed when the result is anything other
   * that -EBUSY.
   */

  do
    {
      /* Start (or re-start) the watchdog timeout */

      ret = wd_start(&spics->dmadog, DMA_TIMEOUT_TICKS,
                     spi_dmatimeout, (wdparm_t)spics);
      if (ret < 0)
        {
           spierr("ERROR: wd_start failed: %d\n", ret);
        }

      /* Wait for the DMA complete */

      ret = nxsem_wait_uninterruptible(&spics->dmawait);

      /* Cancel the watchdog timeout */

      wd_cancel(&spics->dmadog);

      /* Check if we were awakened by an error of some kind. */

      if (ret < 0)
        {
          DEBUGPANIC();
          return;
        }

      /* Not that we might be awakened before the wait is over due to
       * residual counts on the semaphore.  So, to handle, that case,
       * we loop until something changes the DMA result to any value other
       * than -EBUSY.
       */
    }
  while (spics->result == -EBUSY);

  /* Dump the sampled DMA registers */

  spi_dma_sampledone(spics);

  /* Make sure that the DMA is stopped (it will be stopped automatically
   * on normal transfers, but not necessarily when the transfer terminates
   * on an error condition).
   */

  sam_dmastop(spics->rxdma);
  sam_dmastop(spics->txdma);

  /* All we can do is complain if the DMA fails */

  if (spics->result)
    {
      spierr("ERROR: DMA failed with result: %d\n", spics->result);
    }
}
#endif /* CONFIG_SAM34_SPI_DMA */

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer of data to be sent
 *   nwords - the length of data to send from the buffer in number of words.
 *            The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                         size_t nwords)
{
  /* spi_exchange can do this. */

  spi_exchange(dev, buffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to receive data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface.  If nbits <= 8,
 *            the data is packed into uint8_t's; if nbits >8, the data is
 *            packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(struct spi_dev_s *dev, void *buffer, size_t nwords)
{
  /* spi_exchange can do this. */

  spi_exchange(dev, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   cs - Chip select number (identifying the "logical" SPI port)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *sam_spibus_initialize(int port)
{
  struct sam_spidev_s *spi;
  struct sam_spics_s *spics;
  int csno  = (port & __SPI_CS_MASK) >> __SPI_CS_SHIFT;
  int spino = (port & __SPI_SPI_MASK) >> __SPI_SPI_SHIFT;
  irqstate_t flags;
  uint32_t regval;
  unsigned int offset;

  /* The support SAM parts have only a single SPI port */

  spiinfo("port: %d csno: %d spino: %d\n", port, csno, spino);
  DEBUGASSERT(csno >= 0 && csno <= SAM_SPI_NCS);

#if defined(CONFIG_SAM34_SPI0) && defined(CONFIG_SAM34_SPI1)
  DEBUGASSERT(spino >= 0 && spino <= 1);
#elif defined(CONFIG_SAM34_SPI0)
  DEBUGASSERT(spino == 0);
#else
  DEBUGASSERT(spino == 1);
#endif

  /* Allocate a new state structure for this chip select.  NOTE that there
   * is no protection if the same chip select is used in two different
   * chip select structures.
   */

  spics = (struct sam_spics_s *)kmm_zalloc(sizeof(struct sam_spics_s));
  if (!spics)
    {
      spierr("ERROR: Failed to allocate a chip select structure\n");
      return NULL;
    }

  /* Set up the initial state for this chip select structure.  Other fields
   * were zeroed by kmm_zalloc().
   */

#ifdef CONFIG_SAM34_SPI_DMA
  /* Can we do DMA on this peripheral? */

  spics->candma = spino ? SAM34_SPI1_DMA : SAM34_SPI0_DMA;

  /* Pre-allocate DMA channels.  These allocations exploit that fact that
   * SPI0 is managed by DMAC0 and SPI1 is managed by DMAC1.  Hence,
   * the SPI number (spino) is the same as the DMAC number.
   */

  if (spics->candma)
    {
      spics->rxdma = sam_dmachannel(0);
      if (!spics->rxdma)
        {
          spierr("ERROR: Failed to allocate the RX DMA channel\n");
          spics->candma = false;
        }
    }

  if (spics->candma)
    {
      spics->txdma = sam_dmachannel(0);
      if (!spics->txdma)
        {
          spierr("ERROR: Failed to allocate the TX DMA channel\n");
          sam_dmafree(spics->rxdma);
          spics->rxdma  = NULL;
          spics->candma = false;
        }
    }
#endif

  /* Select the SPI operations */

#if defined(CONFIG_SAM34_SPI0) && defined(CONFIG_SAM34_SPI1)
  spics->spidev.ops = spino ? &g_spi1ops : &g_spi0ops;
#elif defined(CONFIG_SAM34_SPI0)
  spics->spidev.ops = &g_spi0ops;
#else
  spics->spidev.ops = &g_spi1ops;
#endif

  /* Save the chip select and SPI controller numbers */

  spics->cs    = csno;
#if defined(CONFIG_SAM34_SPI0) || defined(CONFIG_SAM34_SPI1)
  spics->spino = spino;
#endif

  /* Get the SPI device structure associated with the chip select */

  spi = spi_device(spics);

  /* Has the SPI hardware been initialized? */

  if (!spi->initialized)
    {
      /* Enable clocking to the SPI block */

      flags = enter_critical_section();
#if defined(CONFIG_SAM34_SPI0) && defined(CONFIG_SAM34_SPI1)
      if (spino == 0)
#endif
#if defined(CONFIG_SAM34_SPI0)
        {
          sam_spi0_enableclk();

          /* Configure multiplexed pins as connected on the board.  Chip
           * select pins must be selected by board-specific logic.
           */

          sam_configgpio(GPIO_SPI0_MISO);
          sam_configgpio(GPIO_SPI0_MOSI);
          sam_configgpio(GPIO_SPI0_SPCK);
        }
#endif
#if defined(CONFIG_SAM34_SPI0) && defined(CONFIG_SAM34_SPI1)
      else
#endif
#if defined(CONFIG_SAM34_SPI1)
        {
          sam_spi1_enableclk();

          /* Configure multiplexed pins as connected on the board.  Chip
           * select pins must be selected by board-specific logic.
           */

          sam_configgpio(GPIO_SPI1_MISO);
          sam_configgpio(GPIO_SPI1_MOSI);
          sam_configgpio(GPIO_SPI1_SPCK);
        }
#endif

      /* Disable SPI clocking */

      spi_putreg(spi, SPI_CR_SPIDIS, SAM_SPI_CR_OFFSET);

      /* Execute a software reset of the SPI (twice) */

      spi_putreg(spi, SPI_CR_SWRST, SAM_SPI_CR_OFFSET);
      spi_putreg(spi, SPI_CR_SWRST, SAM_SPI_CR_OFFSET);
      leave_critical_section(flags);

      /* Configure the SPI mode register */

      spi_putreg(spi, SPI_MR_MSTR | SPI_MR_MODFDIS, SAM_SPI_MR_OFFSET);

      /* And enable the SPI */

      spi_putreg(spi, SPI_CR_SPIEN, SAM_SPI_CR_OFFSET);
      up_mdelay(20);

      /* Flush any pending transfers */

      spi_getreg(spi, SAM_SPI_SR_OFFSET);
      spi_getreg(spi, SAM_SPI_RDR_OFFSET);

      /* Initialize the SPI semaphore that enforces mutually exclusive
       * access to the SPI registers.
       */

      nxsem_init(&spi->spisem, 0, 1);
      spi->initialized = true;

#ifdef CONFIG_SAM34_SPI_DMA
      /* Initialize the SPI semaphore that is used to wake up the waiting
       * thread when the DMA transfer completes.  This semaphore is used for
       * signaling and, hence, should not have priority inheritance enabled.
       */

      nxsem_init(&spics->dmawait, 0, 0);
      nxsem_set_protocol(&spics->dmawait, SEM_PRIO_NONE);
#endif

      spi_dumpregs(spi, "After initialization");
    }

  /* Set to mode=0 and nbits=8 and impossible frequency. The SPI will only
   * be reconfigured if there is a change.
   */

  offset = (unsigned int)g_csroffset[csno];
  regval = spi_getreg(spi, offset);
  regval &= ~(SPI_CSR_CPOL | SPI_CSR_NCPHA | SPI_CSR_BITS_MASK);
  regval |= (SPI_CSR_NCPHA | SPI_CSR_BITS(8));
  spi_putreg(spi, regval, offset);

  spics->nbits = 8;
  spiinfo("csr[offset=%02x]=%08x\n", offset, regval);

  return &spics->spidev;
}
#endif /* CONFIG_SAM34_SPI0 || CONFIG_SAM34_SPI1 */
