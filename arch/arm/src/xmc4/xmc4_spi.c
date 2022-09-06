/****************************************************************************
 * arch/arm/src/xmc4/xmc4_spi.c
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
#include <nuttx/wdog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"
#include "xmc4_gpio.h"
#include "xmc4_spi.h"
#include "xmc4_usic.h"
#include "hardware/xmc4_spi.h"
#include "hardware/xmc4_usic.h"
#include "hardware/xmc4_pinmux.h"

#if defined(CONFIG_XMC4_SPI0) || defined(CONFIG_XMC4_SPI1) || \
    defined(CONFIG_XMC4_SPI2) || defined(CONFIG_XMC4_SPI3) || \
    defined(CONFIG_XMC4_SPI4) || defined(CONFIG_XMC4_SPI5)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* When SPI DMA is enabled, small DMA transfers will still be performed by
 * polling logic.  But we need a threshold value to determine what is small.
 * That value is provided by CONFIG_XMC4_SPI_DMATHRESHOLD.
 */

#ifndef CONFIG_XMC4_SPI_DMATHRESHOLD
#  define CONFIG_XMC4_SPI_DMATHRESHOLD 4
#endif

#ifdef CONFIG_XMC4_SPI_DMA
#  if defined(CONFIG_XMC4_SPI0) && defined(CONFIG_XMC4_DMAC0)
#    define XMC4_SPI0_DMA true
#  else
#    define XMC4_SPI0_DMA false
#  endif
#  if defined(CONFIG_XMC4_SPI1) && defined(CONFIG_XMC4_DMAC1)
#    define XMC4_SPI1_DMA true
#  else
#    define XMC4_SPI1_DMA false
#  endif
#  if defined(CONFIG_XMC4_SPI2) && defined(CONFIG_XMC4_DMAC2)
#    define XMC4_SPI2_DMA true
#  else
#    define XMC4_SPI2_DMA false
#  endif
#  if defined(CONFIG_XMC4_SPI3) && defined(CONFIG_XMC4_DMAC3)
#    define XMC4_SPI3_DMA true
#  else
#    define XMC4_SPI3_DMA false
#  endif
#  if defined(CONFIG_XMC4_SPI4) && defined(CONFIG_XMC4_DMAC4)
#    define XMC4_SPI4_DMA true
#  else
#    define XMC4_SPI4_DMA false
#  endif
#  if defined(CONFIG_XMC4_SPI5) && defined(CONFIG_XMC4_DMAC5)
#    define XMC4_SPI5_DMA true
#  else
#    define XMC4_SPI5_DMA false
#  endif
#endif

#ifndef CONFIG_DEBUG_SPI_INFO
#  undef CONFIG_XMC4_SPI_REGDEBUG
#endif

#ifndef CONFIG_DEBUG_DMA_INFO
#  undef CONFIG_XMC4_SPI_DMADEBUG
#endif

/* Clocking *****************************************************************/

/* Select MCU-specific settings */

#if defined(CONFIG_ARCH_CHIP_XMC4)
#  define XMC4_SPI_CLOCK        BOARD_PERIPH_FREQUENCY /* Frequency of the
                                                        * clock */
#else
#  error Unrecognized XMC4xxx architecture
#endif

/* DMA timeout.  The value is not critical; we just don't want the system to
 * hang in the event that a DMA does not finish.  This is set to
 */

#define DMA_TIMEOUT_MS          (800)
#define DMA_TIMEOUT_TICKS       MSEC2TICK(DMA_TIMEOUT_MS)

#define XMC_SPI_OVERSAMPLING    (2UL)

/* Debug ******************************************************************
 * Check if SPI debut is enabled
 */

#ifndef CONFIG_DEBUG_DMA_INFO
#  undef CONFIG_XMC4_SPI_DMADEBUG
#endif

#define DMA_INITIAL         0
#define DMA_AFTER_SETUP     1
#define DMA_AFTER_START     2
#define DMA_CALLBACK        3
#define DMA_TIMEOUT         3
#define DMA_END_TRANSFER    4
#define DMA_NSAMPLES        5

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of the one SPI chip select */

struct xmc4_spics_s
{
  struct spi_dev_s spidev;      /* Externally visible part of SPI interface */

  uint32_t frequency;           /* Requested clock frequency */
  uint8_t mode;                 /* Mode 0,1,2,3 */
  uint8_t nbits;                /* Width of word in bits (8 to 16) */
  uint8_t spino;                /* SPI controller number */
  uint8_t cs;                   /* Chip select number */

#ifdef CONFIG_XMC4_SPI_DMA
  bool candma;                  /* DMA is supported */
  sem_t dmawait;                /* Used to wait for DMA completion */
  struct wdog_s dmadog;         /* Watchdog that handles DMA timeouts */
  int result;                   /* DMA result */
  DMA_HANDLE rxdma;             /* SPI RX DMA handle */
  DMA_HANDLE txdma;             /* SPI TX DMA handle */
#endif

  /* Debug stuff */

#ifdef CONFIG_XMC4_SPI_DMADEBUG
  struct xmc4_dmaregs_s rxdmaregs[DMA_NSAMPLES];
  struct xmc4_dmaregs_s txdmaregs[DMA_NSAMPLES];
#endif
};

/* Type of board-specific SPI status function */

typedef void (*select_t)(struct spi_dev_s *dev, uint32_t devid,
                         bool selected);

/* Chip select register offsets */

/* The overall state of one SPI controller */

struct xmc4_spidev_s
{
  uint32_t base;                /* SPI controller register base address */
  mutex_t spilock;              /* Assures mutually exclusive access to SPI */
  select_t select;              /* SPI select call-out */
  bool initialized;             /* TRUE: Controller has been initialized */
#ifdef CONFIG_XMC4_SPI_DMA
  uint8_t rxintf;               /* RX hardware interface number */
  uint8_t txintf;               /* TX hardware interface number */
#endif

  /* Debug stuff */

#ifdef CONFIG_XMC4_SPI_REGDEBUG
  bool wrlast;                  /* Last was a write */
  uint32_t addresslast;         /* Last address */
  uint32_t valuelast;           /* Last value */
  int ntimes;                   /* Number of times */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

#ifdef CONFIG_XMC4_SPI_REGDEBUG
static bool     spi_checkreg(struct xmc4_spidev_s *spi, bool wr,
                             uint32_t value, uint32_t address);

#else
# define        spi_checkreg(spi, wr, value, address)  (false)
#endif

static inline uint32_t spi_getreg(struct xmc4_spidev_s *spi,
                                  unsigned int offset);
static inline void spi_putreg(struct xmc4_spidev_s *spi, uint32_t value,
                              unsigned int offset);
static inline struct xmc4_spidev_s *spi_device(struct xmc4_spics_s *spics);

#ifdef CONFIG_DEBUG_SPI_INFO
static void     spi_dumpregs(struct xmc4_spidev_s *spi, const char *msg);
#else
# define        spi_dumpregs(spi, msg)
#endif

static inline void spi_flush(struct xmc4_spidev_s *spi);

/* DMA support */

#ifdef CONFIG_XMC4_SPI_DMA

#ifdef CONFIG_XMC4_SPI_DMADEBUG
#  define spi_rxdma_sample(s, i) \
    xmc4_dmasample((s)->rxdma, &(s)->rxdmaregs[i])
#  define spi_txdma_sample(s, i) \
    xmc4_dmasample((s)->txdma, &(s)->txdmaregs[i])
static void     spi_dma_sampleinit(struct xmc4_spics_s *spics);
static void     spi_dma_sampledone(struct xmc4_spics_s *spics);

#else
#  define spi_rxdma_sample(s, i)
#  define spi_txdma_sample(s, i)
#  define spi_dma_sampleinit(s)
#  define spi_dma_sampledone(s)
#endif

static void     spi_rxcallback(DMA_HANDLE handle, void *arg, int result);
static void     spi_txcallback(DMA_HANDLE handle, void *arg, int result);
static inline uintptr_t spi_regaddr(struct xmc4_spics_s *spics,
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

#ifdef CONFIG_XMC4_SPI_DMA
static void     spi_exchange_nodma(struct spi_dev_s *dev,
                                   const void *txbuffer, void *rxbuffer,
                                   size_t nwords);
#endif
static void     spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                             void *rxbuffer, size_t nwords);

#ifndef CONFIG_SPI_EXCHANGE
static void     spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                             size_t nwords);
static void     spi_recvblock(struct spi_dev_s *dev, void *buffer,
                              size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_XMC4_SPI0
/* SPI0 driver operations */

static const struct spi_ops_s g_spi0ops =
{
  .lock         = spi_lock,
  .select       = spi_select,
  .setfrequency = spi_setfrequency,
  .setmode      = spi_setmode,
  .setbits      = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures   = 0,                  /* Not supported */
#endif
  .status       = xmc4_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = xmc4_spi0cmddata,
#endif
  .send         = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = spi_exchange,
#else
  .sndblock     = spi_sndblock,
  .recvblock    = spi_recvblock,
#endif
  .registercallback = 0,              /* Not implemented */
};

/* This is the overall state of the SPI0 controller */

static struct xmc4_spidev_s g_spi0dev =
{
  .base         = XMC4_USIC0_CH0_BASE,
  .select       = xmc4_spi0select,
#ifdef CONFIG_XMC4_SPI_DMA
  .rxintf       = DMACHAN_INTF_SPI0RX,
  .txintf       = DMACHAN_INTF_SPI0TX,
#endif
};
#endif

#ifdef CONFIG_XMC4_SPI1
/* SPI1 driver operations */

static const struct spi_ops_s g_spi1ops =
{
  .lock         = spi_lock,
  .select       = spi_select,
  .setfrequency = spi_setfrequency,
  .setmode      = spi_setmode,
  .setbits      = spi_setbits,
  .status       = xmc4_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = xmc4_spi1cmddata,
#endif
  .send         = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = spi_exchange,
#else
  .sndblock     = spi_sndblock,
  .recvblock    = spi_recvblock,
#endif
  .registercallback = 0,              /* Not implemented */
};

/* This is the overall state of the SPI1 controller */

static struct xmc4_spidev_s g_spi1dev =
{
  .base         = XMC4_USIC0_CH1_BASE,
  .select       = xmc4_spi1select,
#ifdef CONFIG_XMC4_SPI_DMA
  .rxintf       = DMACHAN_INTF_SPI1RX,
  .txintf       = DMACHAN_INTF_SPI1TX,
#endif
};
#endif

#ifdef CONFIG_XMC4_SPI2
/* SPI1 driver operations */

static const struct spi_ops_s g_spi2ops =
{
  .lock         = spi_lock,
  .select       = spi_select,
  .setfrequency = spi_setfrequency,
  .setmode      = spi_setmode,
  .setbits      = spi_setbits,
  .status       = xmc4_spi2status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = xmc4_spi2cmddata,
#endif
  .send         = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = spi_exchange,
#else
  .sndblock     = spi_sndblock,
  .recvblock    = spi_recvblock,
#endif
  .registercallback = 0,              /* Not implemented */
};

/* This is the overall state of the SPI2 controller */

static struct xmc4_spidev_s g_spi2dev =
{
  .base         = XMC4_USIC1_CH0_BASE,
  .select       = xmc4_spi2select,
#ifdef CONFIG_XMC4_SPI_DMA
  .rxintf       = DMACHAN_INTF_SPI2RX,
  .txintf       = DMACHAN_INTF_SPI2TX,
#endif
};
#endif

#ifdef CONFIG_XMC4_SPI3
/* SPI1 driver operations */

static const struct spi_ops_s g_spi3ops =
{
  .lock         = spi_lock,
  .select       = spi_select,
  .setfrequency = spi_setfrequency,
  .setmode      = spi_setmode,
  .setbits      = spi_setbits,
  .status       = xmc4_spi3status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = xmc4_spi3cmddata,
#endif
  .send         = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = spi_exchange,
#else
  .sndblock     = spi_sndblock,
  .recvblock    = spi_recvblock,
#endif
  .registercallback = 0,              /* Not implemented */
};

/* This is the overall state of the SPI3 controller */

static struct xmc4_spidev_s g_spi3dev =
{
  .base         = XMC4_USIC1_CH1_BASE,
  .select       = xmc4_spi3select,
#ifdef CONFIG_XMC4_SPI_DMA
  .rxintf       = DMACHAN_INTF_SPI3RX,
  .txintf       = DMACHAN_INTF_SPI3TX,
#endif
};
#endif

#ifdef CONFIG_XMC4_SPI4
/* SPI1 driver operations */

static const struct spi_ops_s g_spi4ops =
{
  .lock         = spi_lock,
  .select       = spi_select,
  .setfrequency = spi_setfrequency,
  .setmode      = spi_setmode,
  .setbits      = spi_setbits,
  .status       = xmc4_spi4status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = xmc4_spi4cmddata,
#endif
  .send         = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = spi_exchange,
#else
  .sndblock     = spi_sndblock,
  .recvblock    = spi_recvblock,
#endif
  .registercallback = 0,              /* Not implemented */
};

/* This is the overall state of the SPI4 controller */

static struct xmc4_spidev_s g_spi4dev =
{
  .base          = XMC4_USIC2_CH0_BASE,
  .select        = xmc4_spi4select,
#ifdef CONFIG_XMC4_SPI_DMA
  .rxintf        = DMACHAN_INTF_SPI4RX,
  .txintf        = DMACHAN_INTF_SPI4TX,
#endif
};
#endif

#ifdef CONFIG_XMC4_SPI5

/* SPI5 driver operations */

static const struct spi_ops_s g_spi5ops =
{
  .lock         = spi_lock,
  .select       = spi_select,
  .setfrequency = spi_setfrequency,
  .setmode      = spi_setmode,
  .setbits      = spi_setbits,
  .status       = xmc4_spi5status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata      = xmc4_spi5cmddata,
#endif
  .send         = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange     = spi_exchange,
#else
  .sndblock     = spi_sndblock,
  .recvblock    = spi_recvblock,
#endif
  .registercallback = 0,              /* Not implemented */
};

/* This is the overall state of the SPI5 controller */

static struct xmc4_spidev_s g_spi5dev =
{
  .base         = XMC4_USIC2_CH1_BASE,
  .select       = xmc4_spi5select,
#ifdef CONFIG_XMC4_SPI_DMA
  .rxintf       = DMACHAN_INTF_SPI5RX,
  .txintf       = DMACHAN_INTF_SPI5TX,
#endif
};
#endif

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

#ifdef CONFIG_XMC4_SPI_REGDEBUG
static bool spi_checkreg(struct xmc4_spidev_s *spi, bool wr, uint32_t value,
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

      spi->wrlast       = wr;
      spi->valuelast    = value;
      spi->addresslast  = address;
      spi->ntimes       = 0;
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

static inline uint32_t spi_getreg(struct xmc4_spidev_s *spi,
                                  unsigned int offset)
{
  uint32_t  address = spi->base + offset;
  uint32_t  value   = getreg32(address);

#ifdef CONFIG_XMC4_SPI_REGDEBUG
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

static inline void spi_putreg(struct xmc4_spidev_s *spi, uint32_t value,
                              unsigned int offset)
{
  uint32_t address = spi->base + offset;

#ifdef CONFIG_XMC4_SPI_REGDEBUG
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
static void spi_dumpregs(struct xmc4_spidev_s *spi, const char *msg)
{
  spiinfo("%s:\n", msg);
  spiinfo("   KSCFG:%08x    FDR:%08x     BRG:%08x\n",
          getreg32(spi->base + XMC4_USIC_KSCFG_OFFSET),
          getreg32(spi->base + XMC4_USIC_FDR_OFFSET),
          getreg32(spi->base + XMC4_USIC_BRG_OFFSET));
  spiinfo("    INPR:%08x  DX0CR:%08x   DX1CR:%08x\n",
          getreg32(spi->base + XMC4_USIC_INPR_OFFSET),
          getreg32(spi->base + XMC4_USIC_DX0CR_OFFSET),
          getreg32(spi->base + XMC4_USIC_DX1CR_OFFSET));
  spiinfo("   DX2CR:%08x  DX3CR:%08x   DX4CR:%08x\n",
          getreg32(spi->base + XMC4_USIC_DX2CR_OFFSET),
          getreg32(spi->base + XMC4_USIC_DX3CR_OFFSET),
          getreg32(spi->base + XMC4_USIC_DX4CR_OFFSET));
  spiinfo("   DX5CR:%08x   SCTR:%08x    TCSR:%08x\n",
          getreg32(spi->base + XMC4_USIC_DX5CR_OFFSET),
          getreg32(spi->base + XMC4_USIC_SCTR_OFFSET),
          getreg32(spi->base + XMC4_USIC_TCSR_OFFSET));
  spiinfo("     PCR:%08x    CCR:%08x    CMTR:%08x\n",
          getreg32(spi->base + XMC4_USIC_PCR_OFFSET),
          getreg32(spi->base + XMC4_USIC_CCR_OFFSET),
          getreg32(spi->base + XMC4_USIC_CMTR_OFFSET));
  spiinfo("     PSR:%08x   PSCR:%08x  RBUFSR:%08x\n",
          getreg32(spi->base + XMC4_USIC_PSR_OFFSET),
          getreg32(spi->base + XMC4_USIC_PSCR_OFFSET),
          getreg32(spi->base + XMC4_USIC_RBUFSR_OFFSET));
  spiinfo("RBUF01SR:%08x    FMR:%08x   TRBSR:%08x\n",
          getreg32(spi->base + XMC4_USIC_RBUF01SR_OFFSET),
          getreg32(spi->base + XMC4_USIC_FMR_OFFSET),
          getreg32(spi->base + XMC4_USIC_TRBSR_OFFSET));
  spiinfo("   TBCTR:%08x  RBCTR:%08x  TRBPTR:%08x\n",
          getreg32(spi->base + XMC4_USIC_TBCTR_OFFSET),
          getreg32(spi->base + XMC4_USIC_RBCTR_OFFSET),
          getreg32(spi->base + XMC4_USIC_TRBPTR_OFFSET));
  spiinfo("   TRBSR:%08x TRBSCR:%08x   OUTDR:%08x\n",
          getreg32(spi->base + XMC4_USIC_TRBSR_OFFSET),
          getreg32(spi->base + XMC4_USIC_TRBSCR_OFFSET),
          getreg32(spi->base + XMC4_USIC_OUTDR_OFFSET));
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

static inline struct xmc4_spidev_s *spi_device(struct xmc4_spics_s *spics)
{
#if defined(CONFIG_XMC4_SPI0)
  if (spics->spino == 0)
    {
      return &g_spi0dev;
    }
#endif

#if defined(CONFIG_XMC4_SPI1)
  if (spics->spino == 1)
    {
      return &g_spi1dev;
    }
#endif

#if defined(CONFIG_XMC4_SPI2)
  if (spics->spino == 2)
    {
      return &g_spi2dev;
    }
#endif

#if defined(CONFIG_XMC4_SPI3)
  if (spics->spino == 3)
    {
      return &g_spi3dev;
    }
#endif

#if defined(CONFIG_XMC4_SPI4)
  if (spics->spino == 4)
    {
      return &g_spi4dev;
    }
#endif

#if defined(CONFIG_XMC4_SPI5)
  if (spics->spino == 5)
    {
      return &g_spi5dev;
    }
#endif

  /* Otherwise return NULL */

  return NULL;
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

static inline void spi_flush(struct xmc4_spidev_s *spi)
{
  uint32_t regval;

  /* Wait for the transmit buffer/fifo to be "not full." */

  do
    {
      regval = getreg32(spi->base + XMC4_USIC_TCSR_OFFSET);
    }
  while ((regval & USIC_TCSR_TDV) != 0);
}

/****************************************************************************
 * Name: spi_dma_sampleinit
 *
 * Description:
 *   Initialize sampling of DMA registers (if CONFIG_XMC4_SPI_DMADEBUG)
 *
 * Input Parameters:
 *   spics - Chip select doing the DMA
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_XMC4_SPI_DMADEBUG
static void spi_dma_sampleinit(struct xmc4_spics_s *spics)
{
  /* Put contents of register samples into a known state */

  memset(spics->rxdmaregs, 0xff,
         DMA_NSAMPLES * sizeof(struct xmc4_dmaregs_s));
  memset(spics->txdmaregs, 0xff,
         DMA_NSAMPLES * sizeof(struct xmc4_dmaregs_s));

  /* Then get the initial samples */

  xmc4_dmasample(spics->rxdma, &spics->rxdmaregs[DMA_INITIAL]);
  xmc4_dmasample(spics->txdma, &spics->txdmaregs[DMA_INITIAL]);
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

#ifdef CONFIG_XMC4_SPI_DMADEBUG
static void spi_dma_sampledone(struct xmc4_spics_s *spics)
{
  /* Sample the final registers */

  xmc4_dmasample(spics->rxdma, &spics->rxdmaregs[DMA_END_TRANSFER]);
  xmc4_dmasample(spics->txdma, &spics->txdmaregs[DMA_END_TRANSFER]);

  /* Then dump the sampled DMA registers
   * Initial register values
   */

  xmc4_dmadump(spics->txdma, &spics->txdmaregs[DMA_INITIAL],
               "TX: Initial Registers");
  xmc4_dmadump(spics->rxdma, &spics->rxdmaregs[DMA_INITIAL],
               "RX: Initial Registers");

  /* Register values after DMA setup */

  xmc4_dmadump(spics->txdma, &spics->txdmaregs[DMA_AFTER_SETUP],
               "TX: After DMA Setup");
  xmc4_dmadump(spics->rxdma, &spics->rxdmaregs[DMA_AFTER_SETUP],
               "RX: After DMA Setup");

  /* Register values after DMA start */

  xmc4_dmadump(spics->txdma, &spics->txdmaregs[DMA_AFTER_START],
               "TX: After DMA Start");
  xmc4_dmadump(spics->rxdma, &spics->rxdmaregs[DMA_AFTER_START],
               "RX: After DMA Start");

  /* Register values at the time of the TX and RX DMA callbacks
   * -OR- DMA timeout.
   *
   * If the DMA timed out, then there will not be any RX DMA
   * callback samples.  There is probably no TX DMA callback
   * samples either, but we don't know for sure.
   */

  xmc4_dmadump(spics->txdma, &spics->txdmaregs[DMA_CALLBACK],
               "TX: At DMA callback");

  /* Register values at the end of the DMA */

  if (spics->result == -ETIMEDOUT)
    {
      xmc4_dmadump(spics->rxdma, &spics->rxdmaregs[DMA_TIMEOUT],
                   "RX: At DMA timeout");
    }
  else
    {
      xmc4_dmadump(spics->rxdma, &spics->rxdmaregs[DMA_CALLBACK],
                   "RX: At DMA callback");
    }

  xmc4_dmadump(spics->txdma, &spics->txdmaregs[DMA_END_TRANSFER],
               "TX: At End-of-Transfer");
  xmc4_dmadump(spics->rxdma, &spics->rxdmaregs[DMA_END_TRANSFER],
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
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_XMC4_SPI_DMA
static void spi_dmatimeout(wdparm_t arg)
{
  struct xmc4_spics_s *spics = (struct xmc4_spics_s *)arg;

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

#ifdef CONFIG_XMC4_SPI_DMA
static void spi_rxcallback(DMA_HANDLE handle, void *arg, int result)
{
  struct xmc4_spics_s *spics = (struct xmc4_spics_s *)arg;

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
      /* Save the result of the transfer if no error previously reported */

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
 *   arg    - A pointer to the chip select structure
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_XMC4_SPI_DMA
static void spi_txcallback(DMA_HANDLE handle, void *arg, int result)
{
  struct xmc4_spics_s *spics = (struct xmc4_spics_s *)arg;

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

#ifdef CONFIG_XMC4_SPI_DMA
static inline uintptr_t spi_regaddr(struct xmc4_spics_s *spics,
                                    unsigned int offset)
{
  struct xmc4_spidev_s *spi = spi_device(spics);

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
  struct xmc4_spics_s *spics = (struct xmc4_spics_s *)dev;
  struct xmc4_spidev_s *spi = spi_device(spics);
  int ret;

  spiinfo("lock=%d\n", lock);
  if (lock)
    {
      ret = nxmutex_lock(&spi->spilock);
    }
  else
    {
      ret = nxmutex_unlock(&spi->spilock);
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
 *   dev       - Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static void spi_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  struct xmc4_spics_s   *spics  = (struct xmc4_spics_s *)dev;
  struct xmc4_spidev_s  *spi    = spi_device(spics);

  /* Are we selecting or de-selecting the device? */

  spiinfo("selected=%d\n", selected);

  /* Perform any board-specific chip select operations. PIO chip select
   * pins may be programmed by the board specific logic in one of two
   * different ways.  First, the pins may be programmed as SPI peripherals.
   * In that case, the pins are completely controlled by the SPI driver.
   * The xmc4_spi[0|1]select methods still needs to be provided, but they
   * may be only stubs.
   *
   * An alternative way to program the PIO chip select pins is as normal
   * PIO outputs.  In that case, the automatic control of the CS pins is
   * bypassed and this function must provide control of the chip select.
   * NOTE:  In this case, the PIO output pin does *not* have to be the
   * same as the NPCS pin normal associated with the chip select number.
   */

  spi->select(dev, devid, selected);
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
  struct xmc4_spics_s *spics = (struct xmc4_spics_s *)dev;
  int channel = spics->spino;
  int ret;

  spiinfo("cs=%d frequency=%d\n", spics->cs, frequency);

  /* Check if the red frequency is the same as the frequency selection */

  if (spics->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return frequency;
    }

  /* Set SPI frequency (USIC baudrate) */

  ret = xmc4_usic_baudrate(channel, frequency, XMC_SPI_OVERSAMPLING);
  if (ret < 0)
    {
      spierr("Setting frequency to %d failed!\n", frequency);
      return 0;
    }

  /* Save the frequency setting */

  spics->frequency = frequency;

  spiinfo("Frequency configured to %d\n", frequency);

  return frequency;
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
  struct xmc4_spics_s *spics = (struct xmc4_spics_s *)dev;
  struct xmc4_spidev_s *spi = spi_device(spics);
  uint32_t regval;

  spiinfo("cs=%d mode=%d\n", spics->cs, mode);

  /* Has the mode changed? */

  if (mode != spics->mode)
    {
      /* Yes... Set the mode appropriately:
       *
       * SPI  CPOL  CPHA
       * MODE
       *  0    0    0
       *  1    0    1
       *  2    1    0
       *  3    1    1
       */

      regval  = spi_getreg(spi, XMC4_USIC_BRG_OFFSET);
      regval &= ~(USIC_BRG_SCLKCFG_MASK);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          regval |= XMC4_SPI_MODE0;
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          regval |= XMC4_SPI_MODE1;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          regval |= XMC4_SPI_MODE2;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          regval |= XMC4_SPI_MODE3;
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

      spi_putreg(spi, regval, XMC4_USIC_BRG_OFFSET);
      spiinfo("USIC BRG = %08x\n", regval);

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
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct xmc4_spics_s *spics = (struct xmc4_spics_s *)dev;
  struct xmc4_spidev_s *spi = spi_device(spics);
  uint32_t regval;

  spiinfo("cs=%d nbits=%d\n", spics->cs, nbits);
  DEBUGASSERT(nbits > 7 && nbits < 17);

  /* Has the number of bits changed? */

  if (nbits != spics->nbits)
    {
      /* Yes... Configure the new word length */

      regval  = spi_getreg(spi, XMC4_USIC_SCTR_OFFSET);
      regval &= ~(USIC_SCTR_WLE_MASK);
      regval |= USIC_SCTR_WLE(nbits);
      spi_putreg(spi, regval, XMC4_USIC_SCTR_OFFSET);

      spiinfo("SCTR = %08x\n", regval);

      /* Save the selection so that subsequent re-configs will be faster. */

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
 *   function:  (1) One that is enabled only when CONFIG_XMC4_SPI_DMA=y
 *   that performs DMA SPI transfers, but only when a larger block of
 *   data is being transferred.  And (2) another version that does polled
 *   SPI transfers.  When CONFIG_XMC4_SPI_DMA=n the latter is the only
 *   version available; when CONFIG_XMC4_SPI_DMA=y, this version is only
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

#ifdef CONFIG_XMC4_SPI_DMA
static void spi_exchange_nodma(struct spi_dev_s *dev, const void *txbuffer,
                               void *rxbuffer, size_t nwords)
#else
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
#endif
{
  struct xmc4_spics_s *spics = (struct xmc4_spics_s *)dev;
  struct xmc4_spidev_s *spi = spi_device(spics);
  uint32_t regval;
  uint32_t data;
  uint16_t *rxptr16;
  uint16_t *txptr16;
  uint8_t *rxptr8;
  uint8_t *txptr8;

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Set up transmit mode */

  regval  = spi_getreg(spi, XMC4_USIC_CCR_OFFSET);
  regval &= ~USIC_CCR_MODE_MASK;
  regval |= USIC_CCR_MODE_SPI;
  regval |= (USIC_CCR_RIEN | USIC_CCR_AIEN);
  spi_putreg(spi, regval, XMC4_USIC_CCR_OFFSET);

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
   * Note: Good SPI performance would require that we implement
   * DMA transfers!
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

      /* Write the data to transmitted to the Transmit Data Register (TDR) */

      spi_putreg(spi, data, XMC4_USIC_TBUF_OFFSET);

      /* Wait until the last bit be transferred */

      while ((spi_getreg(spi, XMC4_USIC_PSR_OFFSET) &
             (USIC_PSR_TSIF)) == 0)
        {
        }

      spi_putreg(spi, USIC_PSCR_CTSIF, XMC4_USIC_PSCR_OFFSET);

      /* Wait to get some data */

      while ((spi_getreg(spi, XMC4_USIC_PSR_OFFSET) &
             (USIC_PSR_RIF | USIC_PSR_AIF)) == 0)
        {
        }

      spi_putreg(spi, (USIC_PSCR_CRIF | USIC_PSCR_CAIF),
                 XMC4_USIC_PSCR_OFFSET);

      /* Read the received data from the SPI Data Register. */

      data = spi_getreg(spi, XMC4_USIC_RBUF_OFFSET);
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

#ifdef CONFIG_XMC4_SPI_DMA
static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct xmc4_spics_s *spics = (struct xmc4_spics_s *)dev;
  struct xmc4_spidev_s *spi = spi_device(spics);
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

  if (!spics->candma || nbytes <= CONFIG_XMC4_SPI_DMATHRESHOLD)
    {
      spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  spics = (struct xmc4_spics_s *)dev;
  spi   = spi_device(spics);
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

      xmc4_cmcc_invalidate((uintptr_t)rxbuffer,
                           (uintptr_t)rxbuffer + nbytes);

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

  xmc4_dmaconfig(spics->rxdma, rxflags);
  xmc4_dmaconfig(spics->txdma, txflags);

  /* Configure the RX side of the exchange transfer */

  regaddr = spi_regaddr(spics, XMC4_SPI_RDR_OFFSET);
  memaddr = (uintptr_t)rxbuffer;

  ret = xmc4_dmarxsetup(spics->rxdma, regaddr, memaddr, nwords);
  if (ret < 0)
    {
      dmaerr("ERROR: xmc4_dmarxsetup failed: %d\n", ret);
      return;
    }

  spi_rxdma_sample(spics, DMA_AFTER_SETUP);

  /* Configure the TX side of the exchange transfer */

  regaddr = spi_regaddr(spics, XMC4_SPI_TDR_OFFSET);
  memaddr = (uintptr_t)txbuffer;

  ret = xmc4_dmatxsetup(spics->txdma, regaddr, memaddr, nwords);
  if (ret < 0)
    {
      dmaerr("ERROR: xmc4_dmatxsetup failed: %d\n", ret);
      return;
    }

  spi_txdma_sample(spics, DMA_AFTER_SETUP);

  /* Start the DMA transfer */

  spics->result = -EBUSY;
  ret = xmc4_dmastart(spics->rxdma, spi_rxcallback, (void *)spics);
  if (ret < 0)
    {
      dmaerr("ERROR: RX xmc4_dmastart failed: %d\n", ret);
      return;
    }

  spi_rxdma_sample(spics, DMA_AFTER_START);

  ret = xmc4_dmastart(spics->txdma, spi_txcallback, (void *)spics);
  if (ret < 0)
    {
      dmaerr("ERROR: RX xmc4_dmastart failed: %d\n", ret);
      xmc4_dmastop(spics->rxdma);
      return;
    }

  spi_txdma_sample(spics, DMA_AFTER_START);

  /* Wait for DMA completion.  This is done in a loop because there may be
   * false alarm semaphore counts that cause xmc4_wait() not fail to wait
   * or to wake-up prematurely (for example due to the receipt of a signal).
   * We know that the DMA has completed when the result is anything other
   * that -EBUSY.
   */

  do
    {
      /* Start (or re-start) the watchdog timeout */

      ret = wd_start(&spics->dmadog, DMA_TIMEOUT_TICKS,
                     spi_dmatimeout, (wdparm_t)spics);
      if (ret != OK)
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

  xmc4_dmastop(spics->rxdma);
  xmc4_dmastop(spics->txdma);

  /* All we can do is complain if the DMA fails */

  if (spics->result)
    {
      spierr("ERROR: DMA failed with result: %d\n", spics->result);
    }
}
#endif /* CONFIG_XMC4_SPI_DMA */

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
 *   Receive a block of data from SPI
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
 * Name: xmc4_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   channel - USIC channel number (also equal to SPIn).
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *xmc4_spibus_initialize(int channel)
{
  struct xmc4_spidev_s *spi;
  struct xmc4_spics_s *spics;
  int spino = channel;
  irqstate_t flags;
  uint32_t regval;
  int ret;

  spiinfo("channel: %d\n", channel);
  DEBUGASSERT(spino >= 0 && spino <= 5);

  /* Allocate a new state structure for this chip select.  NOTE that there
   * is no protection if the same chip select is used in two different
   * chip select structures.
   */

  spics = (struct xmc4_spics_s *)kmm_zalloc(sizeof(struct xmc4_spics_s));
  if (!spics)
    {
      spierr("ERROR: Failed to allocate a chip select structure\n");
      return NULL;
    }

  /* Set up the initial state for this chip select structure.  Other fields
   * were zeroed by kmm_zalloc().
   */

#ifdef CONFIG_XMC4_SPI_DMA

  /* Can we do DMA on this peripheral? */

  spics->candma = spino ? XMC4_SPI1_DMA : XMC4_SPI0_DMA;

  /* Pre-allocate DMA channels.  These allocations exploit that fact that
   * SPI0 is managed by DMAC0 and SPI1 is managed by DMAC1.  Hence,
   * the SPI number (spino) is the same as the DMAC number.
   */

  if (spics->candma)
    {
      spics->rxdma = xmc4_dmachannel(0);
      if (!spics->rxdma)
        {
          spierr("ERROR: Failed to allocate the RX DMA channel\n");
          spics->candma = false;
        }
    }

  if (spics->candma)
    {
      spics->txdma = xmc4_dmachannel(0);
      if (!spics->txdma)
        {
          spierr("ERROR: Failed to allocate the TX DMA channel\n");
          xmc4_dmafree(spics->rxdma);
          spics->rxdma  = NULL;
          spics->candma = false;
        }
    }
#endif

  /* Select the SPI operations */

#if defined(CONFIG_XMC4_SPI0)
  if (spino == 0)
    {
      spics->spidev.ops = &g_spi0ops;
    }
  else
#endif
#if defined(CONFIG_XMC4_SPI1)
  if (spino == 1)
    {
      spics->spidev.ops = &g_spi1ops;
    }
  else
#endif
#if defined(CONFIG_XMC4_SPI2)
  if (spino == 2)
    {
      spics->spidev.ops = &g_spi2ops;
    }
  else
#endif
#if defined(CONFIG_XMC4_SPI3)
  if (spino == 3)
    {
      spics->spidev.ops = &g_spi3ops;
    }
  else
#endif
#if defined(CONFIG_XMC4_SPI4)
  if (spino == 4)
    {
      spics->spidev.ops = &g_spi4ops;
    }
  else
#endif
#if defined(CONFIG_XMC4_SPI5)
  if (spino == 5)
    {
      spics->spidev.ops = &g_spi5ops;
    }
  else
#endif
    {
      spierr("ERROR:  spino invalid: %d\n", spino);
    }

  /* Save the chip select and SPI controller numbers */

  /* spics->cs    = csno; */

  spics->cs     = 0;
  spics->spino  = spino;

  /* Set to mode=0 and nbits=8 and impossible frequency. The SPI will only
   * be reconfigured if there is a change.
   */

  spics->nbits = 8;

  /* Get the SPI device structure associated with the chip select */

  spi = spi_device(spics);

  /* Has the SPI hardware been initialized? */

  if (!spi->initialized)
    {
      /* Enable clocking to the SPI block */

      flags = enter_critical_section();

      /* Enable the USIC channel */

      ret = xmc4_enable_usic_channel(channel);
      if (ret < 0)
        {
          spierr("ERROR: Failed to enable USIC channel!\n");
          goto errchannel;
        }

#if defined(CONFIG_XMC4_SPI0)
      if (spino == 0)
        {
          /* Configure multiplexed pins as connected on the board.  Chip
           * select pins must be selected by board-specific logic.
           */

          xmc4_gpio_config(GPIO_SPI0_MISO);
          xmc4_gpio_config(GPIO_SPI0_MOSI);
          xmc4_gpio_config(GPIO_SPI0_SCLK);
        }
      else
#endif
#if defined(CONFIG_XMC4_SPI1)
      if (spino == 1)
        {
          /* Configure multiplexed pins as connected on the board.  Chip
           * select pins must be selected by board-specific logic.
           */

          xmc4_gpio_config(GPIO_SPI1_MISO);
          xmc4_gpio_config(GPIO_SPI1_MOSI);
          xmc4_gpio_config(GPIO_SPI1_SCLK);
        }
      else
#endif
#if defined(CONFIG_XMC4_SPI2)
      if (spino == 2)
        {
          /* Configure multiplexed pins as connected on the board.  Chip
           * select pins must be selected by board-specific logic.
           */

          xmc4_gpio_config(GPIO_SPI2_MISO);
          xmc4_gpio_config(GPIO_SPI2_MOSI);
          xmc4_gpio_config(GPIO_SPI2_SCLK);
        }
      else
#endif
#if defined(CONFIG_XMC4_SPI3)
      if (spino == 3)
        {
          /* Configure multiplexed pins as connected on the board.  Chip
           * select pins must be selected by board-specific logic.
           */

          xmc4_gpio_config(GPIO_SPI3_MISO);
          xmc4_gpio_config(GPIO_SPI3_MOSI);
          xmc4_gpio_config(GPIO_SPI3_SCLK);
        }
      else
#endif
#if defined(CONFIG_XMC4_SPI4)
      if (spino == 4)
        {
          /* Configure multiplexed pins as connected on the board.  Chip
           * select pins must be selected by board-specific logic.
           */

          xmc4_gpio_config(GPIO_SPI4_MISO);
          xmc4_gpio_config(GPIO_SPI4_MOSI);
          xmc4_gpio_config(GPIO_SPI4_SCLK);
        }
      else
#endif
#if defined(CONFIG_XMC4_SPI5)
      if (spino == 5)
        {
          /* Configure multiplexed pins as connected on the board.  Chip
           * select pins must be selected by board-specific logic.
           */

          xmc4_gpio_config(GPIO_SPI5_MISO);
          xmc4_gpio_config(GPIO_SPI5_MOSI);
          xmc4_gpio_config(GPIO_SPI5_SCLK);
        }
      else
#endif
        {
          spierr("ERROR:  spino invalid: %d\n", spino);
        }

      /* Leave critical section */

      leave_critical_section(flags);

      /* Set initial clock to 1MHz */

      ret = xmc4_usic_baudrate(channel, 1000000, XMC_SPI_OVERSAMPLING);
      if (ret < 0)
        {
          spierr("Setting initial clock failed!\n");
          goto errchannel;
        }

      /* Set DX0CR input source path and input switch */

      regval  = getreg32(spi->base + XMC4_USIC_DX0CR_OFFSET);
      regval &= ~USIC_DXCR_DSEL_MASK;
      regval |= USIC_DXCR_DSEL_DX(BOARD_SPI_DX);
      regval |= USIC_DXCR_INSW;
      putreg32(regval, spi->base + XMC4_USIC_DX0CR_OFFSET);

      /* Configuration of USIC Shift Control
       * Transmission Mode (TRM)  = 1
       * Passive Data Level (PDL) = 1
       * Serial Direction MSB (SDIR) = 1
       */

      regval = USIC_SCTR_PDL1 | USIC_SCTR_TRM_1LEVEL | USIC_SCTR_SDIR |
               USIC_SCTR_FLE(64) | USIC_SCTR_WLE(spics->nbits);
      spi_putreg(spi, regval, XMC4_USIC_SCTR_OFFSET);

      /* Configuration of USIC Transmit Control/Status Register
       * TBUF Data Enable (TDEN) = 1
       * TBUF Data Single Shot Mode (TDSSM) = 1
       */

      regval = USIC_TCSR_HPCMD | USIC_TCSR_TDEN_TDIV | USIC_TCSR_TDSSM;
      spi_putreg(spi, regval, XMC4_USIC_TCSR_OFFSET);

      /* Configuration of Protocol Control Register */

      regval = USIC_PCR_SSCMODE_MSLSEN | USIC_PCR_SSCMODE_SELCTR |
               USIC_PCR_SSCMODE_FEM | USIC_PCR_SSCMODE_SELINV;
      spi_putreg(spi, regval, XMC4_USIC_PCR_OFFSET);

      /* Define SPI Mode 0 by default */

      regval  = spi_getreg(spi, XMC4_USIC_BRG_OFFSET);
      regval &= ~(USIC_BRG_SCLKCFG_MASK);
      regval |= XMC4_SPI_MODE0;
      spi_putreg(spi, regval, XMC4_USIC_BRG_OFFSET);

      /* Clear protocol status */

      spi_putreg(spi, 0xfffffffful, XMC4_USIC_PSCR_OFFSET);

      /* Disable the parity */

      spi_putreg(spi, 0, XMC4_USIC_CCR_OFFSET);

      /* Initialize the SPI mutex that enforces mutually exclusive
       * access to the SPI registers.
       */

      nxmutex_init(&spi->spilock);
      spi->initialized = true;

#ifdef CONFIG_XMC4_SPI_DMA

      /* Initialize the SPI semaphore that is used to wake up the waiting
       * thread when the DMA transfer completes.  This semaphore is used for
       * signaling and, hence, should not have priority inheritance enabled.
       */

      nxsem_init(&spics->dmawait, 0, 0);
      nxsem_set_protocol(&spics->dmawait, SEM_PRIO_NONE);
#endif

      spi_dumpregs(spi, "After initialization");
    }

  return &spics->spidev;

errchannel:
  kmm_free(spics);
  return NULL;
}
#endif /* CONFIG_XMC4_SPI0 || CONFIG_XMC4_SPI1 */
