/****************************************************************************
 * arch/arm/src/sama5/sam_flexcom_spi.c
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
#include "sam_pio.h"
#include "sam_dmac.h"
#include "sam_memories.h"
#include "sam_periphclks.h"
#include "sam_flexcom_spi.h"
#include "hardware/sam_pmc.h"
#include "hardware/sam_flexcom_spi.h"
#include "hardware/sam_flexcom.h"
#include "hardware/sam_pinmap.h"


#if  defined (CONFIG_SAMA5_FLEXCOM0_SPI) \
  || defined (CONFIG_SAMA5_FLEXCOM1_SPI) \
  || defined (CONFIG_SAMA5_FLEXCOM2_SPI) \
  || defined (CONFIG_SAMA5_FLEXCOM3_SPI) \
  || defined (CONFIG_SAMA5_FLEXCOM4_SPI)


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* When SPI DMA is enabled, small DMA transfers will still be performed by
 * polling logic.  But we need a threshold value to determine what is small.
 * That value is provided by CONFIG_SAMA5_FLEXCOM_SPI_DMATHRESHOLD.
 */

#ifndef CONFIG_SAMA5_FLEXCOM_SPI_DMATHRESHOLD
#  define CONFIG_SAMA5_FLEXCOM_SPI_DMATHRESHOLD 4
#endif

#ifndef CONFIG_DEBUG_SPI_INFO
#  undef CONFIG_SAMA5_SPI_REGDEBUG
#endif

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMA

#if defined(CONFIG_SAMA5_FLEXCOM0_SPI) && defined(CONFIG_SAMA5_XDMAC0)
#    define SAMA5_FLEXCOM0_SPI_DMA true
#  else
#    define SAMA5_FLEXCOM0_SPI_DMA false
#  endif

#if defined(CONFIG_SAMA5_FLEXCOM1_SPI) && defined(CONFIG_SAMA5_XDMAC0)
#    define SAMA5_FLEXCOM1_SPI_DMA true
#  else
#    define SAMA5_FLEXCOM1_SPI_DMA false
#  endif

#if defined(CONFIG_SAMA5_FLEXCOM1_SPI) && defined(CONFIG_SAMA5_XDMAC0)
#    define SAMA5_FLEXCOM2_SPI_DMA true
#  else
#    define SAMA5_FLEXCOM2_SPI_DMA false
#  endif

#if defined(CONFIG_SAMA5_FLEXCOM3_SPI) && defined(CONFIG_SAMA5_XDMAC0)
#    define SAMA5_FLEXCOM3_SPI_DMA true
#  else
#    define SAMA5_FLEXCOM3_SPI_DMA false
#  endif

#if defined(CONFIG_SAMA5_FLEXCOM4_SPI) && defined(CONFIG_SAMA5_XDMAC0)
#    define SAMA5_FLEXCOM4_SPI_DMA true
#  else
#    define SAMA5_FLEXCOM4_SPI_DMA false
#  endif

#endif

#ifndef CONFIG_SAMA5_FLEXCOM_SPI_DMA
#  undef CONFIG_SAMA5_FLEXCOM_SPI_DMADEBUG
#endif

/* Clocking *****************************************************************/

/* Select MCU-specific settings
 *
 * SPI is driven by the main clock.
 */

#define SAM_FLEXCOM_SPI_CLOCK  BOARD_MCK_FREQUENCY

/* DMA timeout.  The value is not critical; we just don't want the system to
 * hang in the event that a DMA does not finish.  This is set to
 */

#define DMA_TIMEOUT_MS    (800)
#define DMA_TIMEOUT_TICKS MSEC2TICK(DMA_TIMEOUT_MS)

/* Debug ********************************************************************/

/* Check if SPI debug is enabled */

#ifndef CONFIG_DEBUG_DMA
#  undef CONFIG_SAMA5_SPI_FLEXCOM_DMADEBUG
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
struct sam_flexcom_spics_s
{
  struct spi_dev_s flexcom_spidev;     /* Externally visible part of the SPI interface */
  uint32_t frequency;          /* Requested clock frequency */
  uint32_t actual;             /* Actual clock frequency */
  uint8_t nbits;               /* Width of word in bits (8 to 16) */
  uint8_t mode;                /* Mode 0,1,2,3 */

#if defined(CONFIG_SAMA5_FLEXCOM0_SPI) || \
    defined(CONFIG_SAMA5_FLEXCOM1_SPI) || \
    defined(CONFIG_SAMA5_FLEXCOM2_SPI) || \
    defined(CONFIG_SAMA5_FLEXCOM3_SPI) || \
    defined(CONFIG_SAMA5_FLEXCOM4_SPI)
  uint8_t flexcom_spino;               /* SPI controller number (0 or 1) */
#endif
  uint8_t cs;                  /* Chip select number */

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMA
  bool candma;                 /* DMA is supported */
  sem_t dmawait;               /* Used to wait for DMA completion */
  struct wdog_s dmadog;        /* Watchdog that handles DMA timeouts */
  int result;                  /* DMA result */
  DMA_HANDLE rxdma;            /* SPI RX DMA handle */
  DMA_HANDLE txdma;            /* SPI TX DMA handle */
#endif

  /* Debug stuff */

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMADEBUG
  struct sam_dmaregs_s rxdmaregs[DMA_NSAMPLES];
  struct sam_dmaregs_s txdmaregs[DMA_NSAMPLES];
#endif
};

/* Type of board-specific SPI status function */

typedef void (*select_t)(uint32_t devid, bool selected);

/* Chip select register offsetrs */

/* The overall state of one SPI controller */

struct sam_flexcom_spidev_s
{
  uint32_t base;               /* SPI controller register base address */
  sem_t flexcom_spisem;                /* Assures mutually exclusive access to SPI */
  select_t select;             /* SPI select callout */
  bool initialized;            /* TRUE: Controller has been initialized */
#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMA
  uint8_t pid;                 /* Peripheral ID */
#endif

  /* Debug stuff */

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_REGDEBUG
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

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_REGDEBUG
static bool     flexcom_spi_checkreg(struct sam_flexcom_spidev_s *flexcom_spi, bool wr,
                  uint32_t value, uint32_t address);
#else
# define        flexcom_spi_checkreg(flexcom_spi,wr,value,address) (false)
#endif

static inline uint32_t flexcom_spi_getreg(struct sam_flexcom_spidev_s *flexcom_spi,
                  unsigned int offset);
static inline void flexcom_spi_putreg(struct sam_flexcom_spidev_s *spi, uint32_t value,
                  unsigned int offset);
static inline struct sam_flexcom_spidev_s *flexcom_spi_device(struct sam_flexcom_spics_s *flexcom_spics);

#ifdef CONFIG_DEBUG_FLEXCOM_SPI_INFO
static void     flexcom_spi_dumpregs(struct sam_flexcom_spidev_s *flexcom_spi, const char *msg);
#else
# define        flexcom_spi_dumpregs(flexcom_spi,msg)
#endif

static inline void flexcom_spi_flush(struct sam_flexcom_spidev_s *flexcom_spi);
static inline uint32_t flexcom_spi_cs2pcs(struct sam_flexcom_spics_s *flexcom_spics);

/* DMA support */

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMA

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMADEBUG
#  define spi_rxdma_sample(s,i) sam_dmasample((s)->rxdma, &(s)->rxdmaregs[i])
#  define spi_txdma_sample(s,i) sam_dmasample((s)->txdma, &(s)->txdmaregs[i])
static void     flexcom_spi_dma_sampleinit(struct sam_flexcom_spics_s *flexcom_spics);
static void     flexcom_spi_dma_sampledone(struct sam_flexcom_spics_s *flexcom_spics);

#else
#  define flexcom_spi_rxdma_sample(s,i)
#  define flexcom_spi_txdma_sample(s,i)
#  define flexcom_spi_dma_sampleinit(s)
#  define flexcom_spi_dma_sampledone(s)

#endif

static void     flexcom_spi_rxcallback(DMA_HANDLE handle, void *arg, int result);
static void     flexcom_spi_txcallback(DMA_HANDLE handle, void *arg, int result);
static inline uintptr_t flexcom_spi_physregaddr(struct sam_flexcom_spics_s *flexcom_spics,
                  unsigned int offset);
#endif

/* SPI methods */

static int      flexcom_spi_lock(struct flexcom_spi_dev_s *dev, bool lock);
static void     flexcom_spi_select(struct flexcom_spi_dev_s *dev, uint32_t devid,
                  bool selected);
static uint32_t flexcom_spi_setfrequency(struct flexcom_spi_dev_s *dev, uint32_t frequency);
static void     flexcom_spi_setmode(struct flexcom_spi_dev_s *dev, enum spi_mode_e mode);
static void     flexcom_spi_setbits(struct flexcom_spi_dev_s *dev, int nbits);
static uint32_t flexcom_spi_send(struct flexcom_spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMA
static void     flexcom_spi_exchange_nodma(struct flexcom_spi_dev_s *dev,
                   const void *txbuffer, void *rxbuffer, size_t nwords);
#endif
static void     flexcom_spi_exchange(struct flexcom_spi_dev_s *dev, const void *txbuffer,
                   void *rxbuffer, size_t nwords);
//#ifndef CONFIG_FLEXCOM_SPI_EXCHANGE
#ifndef CONFIG_SPI_EXCHANGE
static void     flexcom_spi_sndblock(struct flexcom_spi_dev_s *dev,
                   const void *buffer, size_t nwords);
static void     flexcom_spi_recvblock(struct flexcom_spi_dev_s *dev, void *buffer,
                   size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array maps chip select numbers (0-1) to CSR register offsets */

static const uint16_t g_csroffset[2] =
{
  SAM_FLEXCOM_SPI_CSR0_OFFSET, SAM_FLEXCOM_SPI_CSR1_OFFSET
};

#ifdef CONFIG_SAMA5_FLEXCOM0_SPI
/* FLEXCOM0 SPI driver operations */

static const struct spi_ops_s g_flexcom0_spiops =
{
  .lock              = flexcom_spi_lock,
  .select            = flexcom_spi_select,
  .setfrequency      = flexcom_spi_setfrequency,
  .setmode           = flexcom_spi_setmode,
  .setbits           = flexcom_spi_setbits,
#ifdef CONFIG_FLEXCOM0_SPI_HWFEATURES
  .hwfeatures        = 0,                 /* Not supported */
#endif
  .status            = sam_flexcom0_spistatus,
#ifdef CONFIG_FLEXCOM0_SPI_CMDDATA
  .cmddata           = sam_flexcom0_spicmddata,
#endif
  .send              = flexcom_spi_send,
#ifdef CONFIG_FLEXCOM0_SPI_EXCHANGE
  .exchange          = flexcom_spi_exchange,
#else
  .sndblock          = flexcom_spi_sndblock,
  .recvblock         = flexcom_spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

/* This is the overall state of the FLEXCOM0 SPI controller */

static struct sam_flexcom_spidev_s g_flexcom0dev =
{
  .base    = SAM_FLEXCOM0_VBASE,
  .select  = sam_flexcom0_spiselect,
#ifdef SAMA5_FLEXCOM0_SPI_DMA
  .pid     = SAM_PID_FLEXCOM0,
#endif
};
#endif

#ifdef CONFIG_SAMA5_FLEXCOM1_SPI
/* FLEXCOM1 SPI driver operations */

static const struct spi_ops_s g_flexcom1_spiops =
{
  .lock              = flexcom_spi_lock,
  .select            = flexcom_spi_select,
  .setfrequency      = flexcom_spi_setfrequency,
  .setmode           = flexcom_spi_setmode,
  .setbits           = flexcom_spi_setbits,
#ifdef CONFIG_FLEXCOM1_SPI_HWFEATURES
  .hwfeatures        = 0,                 /* Not supported */
#endif
  .status            = sam_flexcom1_spistatus,
#ifdef CONFIG_FLEXCOM1_SPI_CMDDATA
  .cmddata           = sam_flexcom1_spicmddata,
#endif
  .send              = flexcom_spi_send,
#ifdef CONFIG_FLEXCOM1_SPI_EXCHANGE
  .exchange          = flexcom_spi_exchange,
#else
  .sndblock          = flexcom_spi_sndblock,
  .recvblock         = flexcom_spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

/* This is the overall state of the FLEXCOM1 SPI controller */

static struct sam_flexcom_spidev_s g_flexcom1dev =
{
  .base    = SAM_FLEXCOM1_VBASE,
  .select  = sam_flexcom1_spiselect,
#ifdef SAMA5_FLEXCOM1_SPI_DMA
  .pid     = SAM_PID_FLEXCOM1,
#endif
};
#endif

#ifdef CONFIG_SAMA5_FLEXCOM2_SPI
/* FLEXCOM2 SPI driver operations */

static const struct spi_ops_s g_flexcom2_spiops =
{
  .lock              = flexcom_spi_lock,
  .select            = flexcom_spi_select,
  .setfrequency      = flexcom_spi_setfrequency,
  .setmode           = flexcom_spi_setmode,
  .setbits           = flexcom_spi_setbits,
#ifdef CONFIG_FLEXCOM2_SPI_HWFEATURES
  .hwfeatures        = 0,                 /* Not supported */
#endif
  .status            = sam_flexcom2_spistatus,
#ifdef CONFIG_FLEXCOM2_SPI_CMDDATA
  .cmddata           = sam_flexcom2_spicmddata,
#endif
  .send              = flexcom_spi_send,
#ifdef CONFIG_FLEXCOM2_SPI_EXCHANGE
  .exchange          = flexcom_spi_exchange,
#else
  .sndblock          = flexcom_spi_sndblock,
  .recvblock         = flexcom_spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

/* This is the overall state of the FLEXCOM2 SPI controller */

static struct sam_flexcom_spidev_s g_flexcom2dev =
{
  .base    = SAM_FLEXCOM2_VBASE,
  .select  = sam_flexcom2_spiselect,
#ifdef SAMA5_FLEXCOM2_SPI_DMA
  .pid     = SAM_PID_FLEXCOM2,
#endif
};
#endif

#ifdef CONFIG_SAMA5_FLEXCOM3_SPI
/* FLEXCOM3 SPI driver operations */

static const struct spi_ops_s g_flexcom3_spiops =
{
  .lock              = flexcom_spi_lock,
  .select            = flexcom_spi_select,
  .setfrequency      = flexcom_spi_setfrequency,
  .setmode           = flexcom_spi_setmode,
  .setbits           = flexcom_spi_setbits,
#ifdef CONFIG_FLEXCOM3_SPI_HWFEATURES
  .hwfeatures        = 0,                 /* Not supported */
#endif
  .status            = sam_flexcom3_spistatus,
#ifdef CONFIG_FLEXCOM3_SPI_CMDDATA
  .cmddata           = sam_flexcom3_spicmddata,
#endif
  .send              = flexcom_spi_send,
#ifdef CONFIG_FLEXCOM3_SPI_EXCHANGE
  .exchange          = flexcom_spi_exchange,
#else
  .sndblock          = flexcom_spi_sndblock,
  .recvblock         = flexcom_spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

/* This is the overall state of the FLEXCOM3 SPI controller */

static struct sam_flexcom_spidev_s g_flexcom3dev =
{
  .base    = SAM_FLEXCOM3_VBASE,
  .select  = sam_flexcom3_spiselect,
#ifdef SAMA5_FLEXCOM3_SPI_DMA
  .pid     = SAM_PID_FLEXCOM3,
#endif
};
#endif

#ifdef CONFIG_SAMA5_FLEXCOM4_SPI
/* FLEXCOM4 SPI driver operations */

static const struct spi_ops_s g_flexcom4_spiops =
{
  .lock              = flexcom_spi_lock,
  .select            = flexcom_spi_select,
  .setfrequency      = flexcom_spi_setfrequency,
  .setmode           = flexcom_spi_setmode,
  .setbits           = flexcom_spi_setbits,
//#ifdef CONFIG_FLEXCOM4_SPI_HWFEATURES
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                 /* Not supported */
#endif
  .status            = sam_flexcom4_spistatus,
#ifdef CONFIG_SPI_CMDDATA
//#ifdef CONFIG_FLEXCOM4_SPI_CMDDATA
  .cmddata           = sam_flexcom4_spicmddata,
#endif
  .send              = flexcom_spi_send,
//#ifdef CONFIG_FLEXCOM4_SPI_EXCHANGE
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = flexcom_spi_exchange,
#else
  .sndblock          = flexcom_spi_sndblock,
  .recvblock         = flexcom_spi_recvblock,
#endif
  .registercallback  = 0,                 /* Not implemented */
};

/* This is the overall state of the FLEXCOM4 SPI controller */

static struct sam_flexcom_spidev_s g_flexcom4dev =
{
  .base    = SAM_FLEXCOM4_VBASE,
  .select  = sam_flexcom4_spiselect,
#ifdef SAMA5_FLEXCOM4_SPI_DMA
  .pid     = SAM_PID_FLEXCOM4,
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
 * Name: flexcom_spi_checkreg
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

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_REGDEBUG
static bool flexcom_spi_checkreg(struct sam_flexcom_spidev_s *flexcom_spi, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr      == flexcom_spi->wrlast &&     /* Same kind of access? */
      value   == flexcom_spi->valuelast &&  /* Same value? */
      address == flexcom_spi->addresslast)  /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      flexcom_spi->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (flexcom_spi->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          spiinfo("...[Repeats %d times]...\n", flexcom_spi->ntimes);
        }

      /* Save information about the new access */

      flexcom_spi->wrlast      = wr;
      flexcom_spi->valuelast   = value;
      flexcom_spi->addresslast = address;
      flexcom_spi->ntimes      = 0;
    }

  /* Return true if this is the first time that we have done this operation */

  return true;
}
#endif

/****************************************************************************
 * Name: flexcom_spi_getreg
 *
 * Description:
 *  Read an SPI register
 *
 ****************************************************************************/

static inline uint32_t flexcom_spi_getreg(struct sam_flexcom_spidev_s *flexcom_spi,
                                  unsigned int offset)
{
  uint32_t address = flexcom_spi->base + offset;
  uint32_t value = getreg32(address);

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_REGDEBUG
  if (flexcom_spi_checkreg(flexcom_spi, false, value, address))
    {
      spiinfo("%08x->%08x\n", address, value);
    }
#endif

  return value;
}

/****************************************************************************
 * Name: flexcom_spi_putreg
 *
 * Description:
 *  Write a value to an SPI register
 *
 ****************************************************************************/

static inline void flexcom_spi_putreg(struct sam_flexcom_spidev_s *flexcom_spi, uint32_t value,
                              unsigned int offset)
{
  uint32_t address = flexcom_spi->base + offset;

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_REGDEBUG
  if (flexcom_spi_checkreg(flexcom_spi, true, value, address))
    {
      spiinfo("%08x<-%08x\n", address, value);
    }
#endif

  putreg32(value, address);
}

/****************************************************************************
 * Name: flexcom_spi_dumpregs
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

#ifdef CONFIG_DEBUG_FLEXCOM_SPI_INFO
static void flexcom_spi_dumpregs(struct sam_flexcom_spidev_s *flexcom_spi, const char *msg)
{
  spiinfo("%s:\n", msg);
  spiinfo("    MR:%08x   SR:%08x  IMR:%08x\n",
          getreg32(flexcom_spi->base + SAM_FLEXCOM_SPI_MR_OFFSET),
          getreg32(flexcom_spi->base + SAM_FLEXCOM_SPI_SR_OFFSET),
          getreg32(flexcom_spi->base + SAM_FLEXCOM_SPI_IMR_OFFSET));
  spiinfo("  CSR0:%08x CSR1:%08x \n",
          getreg32(flexcom_spi->base + SAM_FLEXCOM_SPI_CSR0_OFFSET),
          getreg32(flexcom_spi->base + SAM_FLEXCOM_SPI_CSR1_OFFSET));
  spiinfo("  WPCR:%08x WPSR:%08x\n",
          getreg32(flexcom_spi->base + SAM_FLEXCOM_SPI_WPCR_OFFSET),
          getreg32(flexcom_spi->base + SAM_FLEXCOM_SPI_WPSR_OFFSET));
}
#endif

/****************************************************************************
 * Name: flexcom_spi_device
 *
 * Description:
 *    Given a chip select instance, return a pointer to the parent SPI
 *    controller instance.
 *
 ****************************************************************************/

static inline struct sam_flexcom_spidev_s *flexcom_spi_device(struct sam_flexcom_spics_s *flexcom_spics)
{
#if defined (CONFIG_SAMA5_FLEXCOM0_SPI)
  if (flexcom_spics->flexcom_spino == 0)
      return &g_flexcom0dev;
#endif
#if defined (CONFIG_SAMA5_FLEXCOM1_SPI)
  if (flexcom_spics->flexcom_spino == 1)
      return &g_flexcom1dev;
#endif
#if defined (CONFIG_SAMA5_FLEXCOM2_SPI)
  if (flexcom_spics->flexcom_spino == 2)
      return &g_flexcom2dev;
#endif
#if defined (CONFIG_SAMA5_FLEXCOM3_SPI)
  if (flexcom_spics->flexcom_spino == 3)
      return &g_flexcom3dev;
#endif
#if defined (CONFIG_SAMA5_FLEXCOM4_SPI)
  if (flexcom_spics->flexcom_spino == 4)
      return &g_flexcom4dev;
#endif 
  /* shouldn't get here */
  DEBUGASSERT(false);
  return NULL;
}

/****************************************************************************
 * Name: flexcom_spi_flush
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

static inline void flexcom_spi_flush(struct sam_flexcom_spidev_s *flexcom_spi)
{
  /* Make sure the no TX activity is in progress... waiting if necessary */

  while ((flexcom_spi_getreg(flexcom_spi, SAM_FLEXCOM_SPI_SR_OFFSET) & FLEXCOM_SPI_INT_TXEMPTY) == 0);

  /* Then make sure that there is no pending RX data .. reading as
   * discarding as necessary.
   */

  while ((flexcom_spi_getreg(flexcom_spi, SAM_FLEXCOM_SPI_SR_OFFSET) & FLEXCOM_SPI_INT_RDRF) != 0)
    {
       flexcom_spi_getreg(flexcom_spi, SAM_FLEXCOM_SPI_RDR_OFFSET);
    }
}

/****************************************************************************
 * Name: flexcom_spi_cs2pcs
 *
 * Description:
 *   Map the chip select number to the bit-set PCS field used in the SPI
 *   registers.  A chip select number is used for indexing and identifying
 *   chip selects.  However, the chip select information is represented by
 *   a bit set in the SPI registers.  This function maps those chip select
 *   numbers to the correct bit set:
 *
 *    CS  Returned Spec  Effective
 *    No.   PCS    Value  NPCS
 *   ---- ------  ------ --------
 *    0    00      x0     10
 *    1    01      01     01
 *
 * Input Parameters:
 *   flexcom_spics - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline uint32_t flexcom_spi_cs2pcs(struct sam_flexcom_spics_s *flexcom_spics)
{
  return ((uint32_t)1 << (flexcom_spics->cs)) - 1;
}

/****************************************************************************
 * Name: flexcom_spi_dma_sampleinit
 *
 * Description:
 *   Initialize sampling of DMA registers (if CONFIG_SAMA5_FLEXCOM_SPI_DMADEBUG)
 *
 * Input Parameters:
 *   flexcom_spics - Chip select doing the DMA
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMADEBUG
static void flexcom_spi_dma_sampleinit(struct sam_flexcom_spics_s *flexcom_spics)
{
  /* Put contents of register samples into a known state */

  memset(flexcom_spics->rxdmaregs, 0xff,
         DMA_NSAMPLES * sizeof(struct sam_dmaregs_s));
  memset(flexcom_spics->txdmaregs, 0xff,
         DMA_NSAMPLES * sizeof(struct sam_dmaregs_s));

  /* Then get the initial samples */

  sam_dmasample(flexcom_spics->rxdma, &flexcom_spics->rxdmaregs[DMA_INITIAL]);
  sam_dmasample(flexcom_spics->txdma, &flexcom_spics->txdmaregs[DMA_INITIAL]);
}
#endif

/****************************************************************************
 * Name: flexcom_spi_dma_sampledone
 *
 * Description:
 *   Dump sampled DMA registers
 *
 * Input Parameters:
 *   flexcom_spics - Chip select doing the DMA
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMADEBUG
static void flexcom_spi_dma_sampledone(struct sam_flexcom_spics_s *flexcom_spics)
{
  /* Sample the final registers */

  sam_dmasample(flexcom_spics->rxdma, &flexcom_spics->rxdmaregs[DMA_END_TRANSFER]);
  sam_dmasample(flexcom_spics->txdma, &flexcom_spics->txdmaregs[DMA_END_TRANSFER]);

  /* Then dump the sampled DMA registers */

  /* Initial register values */

  sam_dmadump(flexcom_spics->txdma, &flexcom_spics->txdmaregs[DMA_INITIAL],
              "TX: Initial Registers");
  sam_dmadump(flexcom_spics->rxdma, &flexcom_spics->rxdmaregs[DMA_INITIAL],
              "RX: Initial Registers");

  /* Register values after DMA setup */

  sam_dmadump(flexcom_spics->txdma, &flexcom_spics->txdmaregs[DMA_AFTER_SETUP],
              "TX: After DMA Setup");
  sam_dmadump(flexcom_spics->rxdma, &flexcom_spics->rxdmaregs[DMA_AFTER_SETUP],
              "RX: After DMA Setup");

  /* Register values after DMA start */

  sam_dmadump(flexcom_spics->txdma, &flexcom_spics->txdmaregs[DMA_AFTER_START],
              "TX: After DMA Start");
  sam_dmadump(flexcom_spics->rxdma, &flexcom_spics->rxdmaregs[DMA_AFTER_START],
              "RX: After DMA Start");

  /* Register values at the time of the TX and RX DMA callbacks
   * -OR- DMA timeout.
   *
   * If the DMA timedout, then there will not be any RX DMA
   * callback samples.  There is probably no TX DMA callback
   * samples either, but we don't know for sure.
   */

  sam_dmadump(flexcom_spics->txdma, &flexcom_spics->txdmaregs[DMA_CALLBACK],
              "TX: At DMA callback");

  /* Register values at the end of the DMA */

  if (flexcom_spics->result == -ETIMEDOUT)
    {
      sam_dmadump(flexcom_spics->rxdma, &flexcom_spics->rxdmaregs[DMA_TIMEOUT],
                  "RX: At DMA timeout");
    }
  else
    {
      sam_dmadump(flexcom_spics->rxdma, &flexcom_spics->rxdmaregs[DMA_CALLBACK],
                  "RX: At DMA callback");
    }

  sam_dmadump(flexcom_spics->rxdma, &flexcom_spics->rxdmaregs[DMA_END_TRANSFER],
              "RX: At End-of-Transfer");
  sam_dmadump(flexcom_spics->txdma, &flexcom_spics->txdmaregs[DMA_END_TRANSFER],
              "TX: At End-of-Transfer");
}
#endif

/****************************************************************************
 * Name: flexcom_spi_dmatimeout
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

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMA
static void flexcom_spi_dmatimeout(wdparm_t arg)
{
  struct sam_flexcom_spics_s *flexcom_spics = (struct sam_flexcom_spics_s *)arg;
  DEBUGASSERT(flexcom_spics != NULL);

  /* Sample DMA registers at the time of the timeout */

  flexcom_spi_rxdma_sample(flexcom_spics, DMA_CALLBACK);

  /* Report timeout result, perhaps overwriting any failure reports from
   * the TX callback.
   */

  flexcom_spics->result = -ETIMEDOUT;

  /* Then wake up the waiting thread */

  nxsem_post(&flexcom_spics->dmawait);
}
#endif

/****************************************************************************
 * Name: flexcom_spi_rxcallback
 *
 * Description:
 *   This callback function is invoked at the completion of the SPI RX DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   arg - A pointer to the chip select struction
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMA
static void flexcom_spi_rxcallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_flexcom_spics_s *flexcom_spics = (struct sam_flexcom_spics_s *)arg;
  DEBUGASSERT(flexcom_spics != NULL);

  /* Cancel the watchdog timeout */

  wd_cancel(&flexcom_spics->dmadog);

  /* Sample DMA registers at the time of the callback */

  flexcom_spi_rxdma_sample(flexcom_spics, DMA_CALLBACK);

  /* Report the result of the transfer only if the TX callback has not
   * already reported an error.
   */

  if (flexcom_spics->result == -EBUSY)
    {
      /* Save the result of the transfer if no error was previously
       * reported
       */

      flexcom_spics->result = result;
    }

  /* Then wake up the waiting thread */

  nxsem_post(&flexcom_spics->dmawait);
}
#endif

/****************************************************************************
 * Name: flexcom_spi_txcallback
 *
 * Description:
 *   This callback function is invoked at the completion of the SPI TX DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   arg - A pointer to the chip select struction
 *   result - The result of the DMA transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMA
static void flexcom_spi_txcallback(DMA_HANDLE handle, void *arg, int result)
{
  struct sam_flexcom_spics_s *flexcom_spics = (struct sam_spics_s *)arg;
  DEBUGASSERT(flexcom_spics != NULL);

  flexcom_spi_txdma_sample(flexcom_spics, DMA_CALLBACK);

  /* Do nothing on the TX callback unless an error is reported.  This
   * callback is not really important because the SPI exchange is not
   * complete until the RX callback is received.
   */

  if (result != OK && flexcom_spics->result == -EBUSY)
    {
      /* Save the result of the transfer if an error is reported */

      flexcom_spics->result = result;
    }
}
#endif

/****************************************************************************
 * Name: flexcom_spi_physregaddr
 *
 * Description:
 *   Return the physical address of an SPI register
 *
 ****************************************************************************/

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMA
static inline uintptr_t flexcom_spi_physregaddr(struct sam_flexcom_spics_s *flexcom_spics,
                                        unsigned int offset)
{
  struct sam_flexcom_spidev_s *flexcom_spi = flexcom_spi_device(flexcom_spics);
  return sam_physregaddr(flexcom_spi->base + offset);
}
#endif

/****************************************************************************
 * Name: flexcom_spi_lock
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

static int flexcom_spi_lock(struct flexcom_spi_dev_s *flexcom_dev, bool lock)
{
  struct sam_flexcom_spics_s *flexcom_spics = (struct sam_flexcom_spics_s *)flexcom_dev;
  struct sam_flexcom_spidev_s *flexcom_spi = flexcom_spi_device(flexcom_spics);
  int ret;

  spiinfo("lock=%d\n", lock);
  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&flexcom_spi->flexcom_spisem);
    }
  else
    {
      ret = nxsem_post(&flexcom_spi->flexcom_spisem);
    }

  return ret;
}

/****************************************************************************
 * Name: flexcom_spi_select
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

static void flexcom_spi_select(struct flexcom_spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  struct sam_flexcom_spics_s *flexcom_spics = (struct sam_flexcom_spics_s *)dev;
  struct sam_flexcom_spidev_s *flexcom_spi = flexcom_spi_device(flexcom_spics);
  uint32_t regval;

  /* Are we selecting or de-selecting the device? */

  spiinfo("selected=%d\n", selected);
  if (selected)
    {
      spiinfo("cs=%d\n", flexcom_spics->cs);

      /* Before writing the TDR, the PCS field in the SPI_MR register must be
       * set in order to select a slave.
       */

      regval  = flexcom_spi_getreg(flexcom_spi, SAM_FLEXCOM_SPI_MR_OFFSET);
      regval &= ~FLEXCOM_SPI_MR_PCS_MASK;
      regval |= (flexcom_spi_cs2pcs(flexcom_spics) << FLEXCOM_SPI_MR_PCS_SHIFT);
      flexcom_spi_putreg(flexcom_spi, regval, SAM_FLEXCOM_SPI_MR_OFFSET);
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

  flexcom_spi->select(devid, selected);
}

/****************************************************************************
 * Name: flexcom_spi_setfrequency
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

static uint32_t flexcom_spi_setfrequency(struct flexcom_spi_dev_s *dev, uint32_t frequency)
{
  struct sam_flexcom_spics_s *flexcom_spics = (struct sam_flexcom_spics_s *)dev;
  struct sam_flexcom_spidev_s *flexcom_spi = flexcom_spi_device(flexcom_spics);
  uint32_t actual;
  uint32_t scbr;
  uint32_t dlybs;
  uint32_t dlybct;
  uint32_t regval;
  unsigned int offset;

  spiinfo("cs=%d frequency=%" PRId32 "\n", flexcom_spics->cs, frequency);

  /* Check if the requested frequency is the same as the frequency
   * selection
   */

  if (flexcom_spics->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return flexcom_spics->actual;
    }

  /* Configure SPI to a frequency as close as possible to the requested
   * frequency.
   *
   *   SPCK frequency = SPI_CLK / SCBR, or SCBR = SPI_CLK / frequency
   */

  scbr = SAM_FLEXCOM_SPI_CLOCK / frequency;

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

  offset = (unsigned int)g_csroffset[flexcom_spics->cs];
  regval  = flexcom_spi_getreg(flexcom_spi, offset);
  regval &= ~(FLEXCOM_SPI_CSR_SCBR_MASK | FLEXCOM_SPI_CSR_DLYBS_MASK | FLEXCOM_SPI_CSR_DLYBCT_MASK);
  regval |= scbr << FLEXCOM_SPI_CSR_SCBR_SHIFT;

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

  dlybs   = SAM_FLEXCOM_SPI_CLOCK / 500000;
  regval |= dlybs << FLEXCOM_SPI_CSR_DLYBS_SHIFT;

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

  dlybct  = SAM_FLEXCOM_SPI_CLOCK / 200000 / 32;
  regval |= dlybct << FLEXCOM_SPI_CSR_DLYBCT_SHIFT;
  flexcom_spi_putreg(flexcom_spi, regval, offset);

  /* Calculate the new actual frequency */

  actual = SAM_FLEXCOM_SPI_CLOCK / scbr;
  spiinfo("csr[offset=%02x]=%08" PRIx32 " actual=%" PRId32 "\n",
          offset, regval, actual);

  /* Save the frequency setting */

  flexcom_spics->frequency = frequency;
  flexcom_spics->actual    = actual;

  spiinfo("Frequency %" PRId32 "->%" PRId32 "\n", frequency, actual);
  return actual;
}

/****************************************************************************
 * Name: flexcom_spi_setmode
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

static void flexcom_spi_setmode(struct flexcom_spi_dev_s *dev, enum spi_mode_e mode)
{
  struct sam_flexcom_spics_s *flexcom_spics = (struct sam_flexcom_spics_s *)dev;
  struct sam_flexcom_spidev_s *flexcom_spi = flexcom_spi_device(flexcom_spics);
  uint32_t regval;
  unsigned int offset;

  spiinfo("cs=%d mode=%d\n", flexcom_spics->cs, mode);

  /* Has the mode changed? */

  if (mode != flexcom_spics->mode)
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

      offset = (unsigned int)g_csroffset[flexcom_spics->cs];
      regval  = flexcom_spi_getreg(flexcom_spi, offset);
      regval &= ~(FLEXCOM_SPI_CSR_CPOL | FLEXCOM_SPI_CSR_NCPHA);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; NCPHA=1 */
          regval |= FLEXCOM_SPI_CSR_NCPHA;
          break;

        case SPIDEV_MODE1: /* CPOL=0; NCPHA=0 */
          break;

        case SPIDEV_MODE2: /* CPOL=1; NCPHA=1 */
          regval |= (FLEXCOM_SPI_CSR_CPOL | FLEXCOM_SPI_CSR_NCPHA);
          break;

        case SPIDEV_MODE3: /* CPOL=1; NCPHA=0 */
          regval |= FLEXCOM_SPI_CSR_CPOL;
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

      flexcom_spi_putreg(flexcom_spi, regval, offset);
      spiinfo("csr[offset=%02x]=%08" PRIx32 "\n", offset, regval);

      /* Save the mode so that subsequent re-configurations will be faster */

      flexcom_spics->mode = mode;
    }
}

/****************************************************************************
 * Name: flexcom_spi_setbits
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

static void flexcom_spi_setbits(struct flexcom_spi_dev_s *dev, int nbits)
{
  struct sam_flexcom_spics_s *flexcom_spics = (struct sam_flexcom_spics_s *)dev;
  struct sam_flexcom_spidev_s *flexcom_spi = flexcom_spi_device(flexcom_spics);
  uint32_t regval;
  unsigned int offset;

  spiinfo("cs=%d nbits=%d\n", flexcom_spics->cs, nbits);
  DEBUGASSERT(nbits > 7 && nbits < 17);

  /* NOTE:  The logic in spi_send and in spi_exchange only handles 8-bit
   * data at the present time.  So the following extra assertion is a
   * reminder that we have to fix that someday.
   */

  DEBUGASSERT(nbits == 8); /* Temporary -- FIX ME */

  /* Has the number of bits changed? */

  if (nbits != flexcom_spics->nbits)
    {
      /* Yes... Set number of bits appropriately */

      offset  = (unsigned int)g_csroffset[flexcom_spics->cs];
      regval  = flexcom_spi_getreg(flexcom_spi, offset);
      regval &= ~FLEXCOM_SPI_CSR_BITS_MASK;
      regval |= FLEXCOM_SPI_CSR_BITS(nbits);
      flexcom_spi_putreg(flexcom_spi, regval, offset);

      spiinfo("csr[offset=%02x]=%08" PRIx32 "\n", offset, regval);

      /* Save the selection so that subsequent re-configurations will be
       * faster.
       */

      flexcom_spics->nbits = nbits;
    }
}

/****************************************************************************
 * Name: flexcom_spi_send
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

static uint32_t flexcom_spi_send(struct flexcom_spi_dev_s *dev, uint32_t wd)
{
  uint8_t txbyte;
  uint8_t rxbyte;

  /* spi_exchange can do this. Note: right now, this only deals with 8-bit
   * words.  If the SPI interface were configured for words of other sizes,
   * this would fail.
   */

  txbyte = (uint8_t)wd;
  rxbyte = (uint8_t)0;
  flexcom_spi_exchange(dev, &txbyte, &rxbyte, 1);

  spiinfo("Sent %02x received %02x\n", txbyte, rxbyte);
  return (uint32_t)rxbyte;
}

/****************************************************************************
 * Name: flexcom_spi_exchange (and flexcom_spi_exchange_nodma)
 *
 * Description:
 *   Exchange a block of data from SPI.  There are two versions of this
 *   function:  (1) One that is enabled only when CONFIG_SAMA5_FLEXCOM_SPI_DMA=y
 *   that performs DMA SPI transfers, but only when a larger block of
 *   data is being transferred.  And (2) another version that does polled
 *   SPI transfers.  When CONFIG_SAMA5_SPI_FLEXCOM_DMA=n the latter is the only
 *   version available; when CONFIG_SAMA5_FLEXCOM_SPI_DMA=y, this version is only
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

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMA
static void spi_exchange_nodma(struct flexcom_spi_dev_s *dev, const void *txbuffer,
              void *rxbuffer, size_t nwords)
#else
static void flexcom_spi_exchange(struct flexcom_spi_dev_s *dev, const void *txbuffer,
              void *rxbuffer, size_t nwords)
#endif
{
  struct sam_flexcom_spics_s *flexcom_spics = (struct sam_flexcom_spics_s *)dev;
  struct sam_flexcom_spidev_s *flexcom_spi = flexcom_spi_device(flexcom_spics);
  uint8_t *rxptr = (uint8_t *)rxbuffer;
  uint8_t *txptr = (uint8_t *)txbuffer;
  uint32_t pcs;
  uint32_t data;

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Set up PCS bits */

  pcs = flexcom_spi_cs2pcs(flexcom_spics) << FLEXCOM_SPI_TDR_PCS_SHIFT;

  /* Make sure that any previous transfer is flushed from the hardware */

  flexcom_spi_flush(flexcom_spi);

  /* Loop, sending each word in the user-provided data buffer.
   *
   * Note 1: Right now, this only deals with 8-bit words.  If the SPI
   *         interface were configured for words of other sizes, this
   *         would fail.
   * Note 2: Good SPI performance would require that we implement DMA
   *         transfers!
   * Note 3: This loop might be made more efficient.  Would logic
   *         like the following improve the throughput?  Or would it
   *         just add the risk of overruns?
   *
   *   Get word 1;
   *   Send word 1;  Now word 1 is "in flight"
   *   nwords--;
   *   for (; nwords > 0; nwords--)
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
      /* Get the data to send (0xff if there is no data source) */

      if (txptr)
        {
          data = (uint32_t)*txptr++;
        }
      else
        {
          data = 0xffff;
        }

      /* Set the PCS field in the value written to the TDR */

      data |= pcs;

      /* Do we need to set the LASTXFER bit in the TDR value too? */

#ifdef CONFIG_FLEXCOM_SPI_VARSELECT
      if (nwords == 1)
        {
          data |= FLEXCOM_SPI_TDR_LASTXFER;
        }
#endif

      /* Wait for any previous data written to the TDR to be transferred
       * to the serializer.
       */

      while ((flexcom_spi_getreg(flexcom_spi, SAM_FLEXCOM_SPI_SR_OFFSET) & FLEXCOM_SPI_INT_TDRE) == 0);

      /* Write the data to transmitted to the Transmit Data Register (TDR) */

      flexcom_spi_putreg(flexcom_spi, data, SAM_FLEXCOM_SPI_TDR_OFFSET);

      /* Wait for the read data to be available in the RDR.
       * TODO:  Data transfer rates would be improved using the RX FIFO
       *        (and also DMA)
       */

      while ((flexcom_spi_getreg(flexcom_spi, SAM_FLEXCOM_SPI_SR_OFFSET) & FLEXCOM_SPI_INT_RDRF) == 0);

      /* Read the received data from the SPI Data Register..
       * TODO: The following only works if nbits <= 8.
       */

      data = flexcom_spi_getreg(flexcom_spi, SAM_FLEXCOM_SPI_RDR_OFFSET);
      if (rxptr)
        {
          *rxptr++ = (uint8_t)data;
        }
    }
}

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMA
static void flexcom_spi_exchange(struct flexcom_spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct sam_flexcom_spics_s *flexcom_spics = (struct sam_flexcom_spics_s *)dev;
  struct sam_flexcom_spidev_s *flexcom_spi = flexcom_spi_device(flexcom_spics);
  uint32_t rxflags;
  uint32_t txflags;
  uint32_t txdummy;
  uint32_t rxdummy;
  uint32_t paddr;
  uint32_t maddr;
  int ret;

  /* If we cannot do DMA -OR- if this is a small SPI transfer, then let
   * spi_exchange_nodma() do the work.
   */

  if (!flexcom_spics->candma || nwords <= CONFIG_SAMA5_FLEXCOM_SPI_DMATHRESHOLD)
    {
      flexcom_spi_exchange_nodma(dev, txbuffer, rxbuffer, nwords);
      return;
    }

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  flexcom_spics = (struct flexcom_sam_spics_s *)dev;
  flexcom_spi = flexcom_spi_device(flexcom_spics);
  DEBUGASSERT(flexcom_spics && flexcom_spi);

  /* Make sure that any previous transfer is flushed from the hardware */

  flexcom_spi_flush(flexcom_spi);

  /* Sample initial DMA registers */

  flexcom_spi_dma_sampleinit(flexcom_spics);

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

  rxflags = DMACH_FLAG_FIFOCFG_LARGEST | DMACH_FLAG_PERIPHPID(flexcom_spi->pid) |
            DMACH_FLAG_PERIPHH2SEL | DMACH_FLAG_PERIPHISPERIPH |
            DMACH_FLAG_PERIPHAHB_AHB_IF2 | DMACH_FLAG_PERIPHWIDTH_8BITS |
            DMACH_FLAG_PERIPHCHUNKSIZE_1 | DMACH_FLAG_MEMPID_MAX |
            DMACH_FLAG_MEMAHB_AHB_IF0 | DMACH_FLAG_MEMWIDTH_8BITS |
            DMACH_FLAG_MEMCHUNKSIZE_1 | DMACH_FLAG_MEMBURST_4;

  if (!rxbuffer)
    {
      /* No sink data buffer.  Point to our dummy buffer and leave
       * the rxflags so that no address increment is performed.
       */

      rxbuffer = (void *)&rxdummy;
    }
  else
    {
      /* A receive buffer is available.  Use normal TX memory incrementing. */

      rxflags |= DMACH_FLAG_MEMINCREMENT;
    }

  txflags = DMACH_FLAG_FIFOCFG_LARGEST | DMACH_FLAG_PERIPHPID(flexcom_spi->pid) |
            DMACH_FLAG_PERIPHH2SEL | DMACH_FLAG_PERIPHISPERIPH |
            DMACH_FLAG_PERIPHAHB_AHB_IF2 | DMACH_FLAG_PERIPHWIDTH_8BITS |
            DMACH_FLAG_PERIPHCHUNKSIZE_1 | DMACH_FLAG_MEMPID_MAX |
            DMACH_FLAG_MEMAHB_AHB_IF0 | DMACH_FLAG_MEMWIDTH_8BITS |
            DMACH_FLAG_MEMCHUNKSIZE_1 | DMACH_FLAG_MEMBURST_4;

  if (!txbuffer)
    {
      /* No source data buffer.  Point to our dummy buffer and configure
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

  sam_dmaconfig(flexcom_spics->rxdma, rxflags);
  sam_dmaconfig(flexcom_spics->txdma, txflags);

  /* Configure the exchange transfers */

  paddr = flexcom_spi_physregaddr(flexcom_spics, SAM_FLEXCOM_SPI_RDR_OFFSET);
  maddr = sam_physramaddr((uintptr_t)rxbuffer);

  ret = sam_dmarxsetup(flexcom_spics->rxdma, paddr, maddr, nwords);
  if (ret < 0)
    {
      dmaerr("ERROR: sam_dmarxsetup failed: %d\n", ret);
      return;
    }

  flexcom_spi_rxdma_sample(flexcom_spics, DMA_AFTER_SETUP);

  paddr = flexcom_spi_physregaddr(flexcom_spics, SAM_FLEXCOM_SPI_TDR_OFFSET);
  maddr = sam_physramaddr((uintptr_t)txbuffer);

  ret = sam_dmatxsetup(flexcom_spics->txdma, paddr, maddr, nwords);
  if (ret < 0)
    {
      dmaerr("ERROR: sam_dmatxsetup failed: %d\n", ret);
      return;
    }

  flexcom_spi_txdma_sample(flexcom_spics, DMA_AFTER_SETUP);

  /* Start the DMA transfer */

  flexcom_spics->result = -EBUSY;
  ret = sam_dmastart(flexcom_spics->rxdma, flexcom_spi_rxcallback, (void *)flexcom_spics);
  if (ret < 0)
    {
      dmaerr("ERROR: RX sam_dmastart failed: %d\n", ret);
      return;
    }

  flexcom_spi_rxdma_sample(flexcom_spics, DMA_AFTER_START);

  ret = sam_dmastart(flexcom_spics->txdma, flexcom_spi_txcallback, (void *)flexcom_spics);
  if (ret < 0)
    {
      dmaerr("ERROR: RX sam_dmastart failed: %d\n", ret);
      sam_dmastop(flexcom_spics->rxdma);
      return;
    }

  flexcom_spi_txdma_sample(spics, DMA_AFTER_START);

  /* Wait for DMA completion.  This is done in a loop because there my be
   * false alarm semaphore counts that cause sam_wait() not fail to wait
   * or to wake-up prematurely (for example due to the receipt of a signal).
   * We know that the DMA has completed when the result is anything other
   * that -EBUSY.
   */

  do
    {
      /* Start (or re-start) the watchdog timeout */

      ret = wd_start(&flexcom_spics->dmadog, DMA_TIMEOUT_TICKS,
                     flexcom_spi_dmatimeout, (wdparm_t)flexcom_spics);
      if (ret < 0)
        {
           spierr("ERROR: wd_start failed: %d\n", ret);
        }

      /* Wait for the DMA complete */

      ret = nxsem_wait_uninterruptible(&flexcom_spics->dmawait);

      /* Cancel the watchdog timeout */

      wd_cancel(&flexcom_spics->dmadog);

      /* Check if we were awakened by an error of some kind. */

      if (ret < 0)
        {
          DEBUGPANIC();
          return;
        }

      /* Not that we might be awkened before the wait is over due to
       * residual counts on the semaphore.  So, to handle, that case,
       * we loop until something changes the DMA result to any value other
       * than -EBUSY.
       */
    }
  while (flexcom_spics->result == -EBUSY);

  /* Dump the sampled DMA registers */

  flexcom_spi_dma_sampledone(flexcom_spics);

  /* Make sure that the DMA is stopped (it will be stopped automatically
   * on normal transfers, but not necessarily when the transfer terminates
   * on an error condition).
   */

  sam_dmastop(flexcom_spics->rxdma);
  sam_dmastop(flexcom_spics->txdma);

  /* All we can do is complain if the DMA fails */

  if (flexcom_spics->result)
    {
      spierr("ERROR: DMA failed with result: %d\n", spics->result);
    }
}
#endif /* CONFIG_SAMA5_FLEXCOM_SPI_DMA */

/****************************************************************************
 * Name: flexcom_spi_sndblock
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

#ifndef CONFIG_FLEXCOM_SPI_EXCHANGE
static void flexcom_spi_sndblock(struct flexcom_spi_dev_s *dev, const void *buffer,
                         size_t nwords)
{
  /* spi_exchange can do this. */

  flexcom_spi_exchange(dev, buffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: flexcom_spi_recvblock
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

#ifndef CONFIG_FLEXCOM_SPI_EXCHANGE
static void flexcom_spi_recvblock(struct flexcom_spi_dev_s *dev, void *buffer,
                          size_t nwords)
{
  /* spi_exchange can do this. */

  flexcom_spi_exchange(dev, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_flexcom_spibus_initialize
 *
 * Description:
 *   Initialize the selected flexcom SPI port
 *
 * Input Parameters:
 *   port - the 5 flexcom ports only have 2 physial CS lines
 *        - so their are 10 "logical" ports.
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct flexcom_spi_dev_s *sam_flexcom_spibus_initialize(int port)
{
  struct sam_flexcom_spidev_s *flexcom_spi;
  struct sam_flexcom_spics_s *flexcom_spics;
  int flexcom_csno  = (port & __FLEXCOM_SPI_CS_MASK) >> __FLEXCOM_SPI_CS_SHIFT;
  int flexcom_spino = (port & __FLEXCOM_SPI_SPI_MASK) >> __FLEXCOM_SPI_SPI_SHIFT;
  irqstate_t flags;
  uint32_t regval;
  unsigned int offset;

  spiinfo("port: %d flexcom_csno: %d spino: %d\n", port, flexcom_csno, flexcom_spino);
  DEBUGASSERT(flexcom_csno >= 0 && flexcom_csno < SAM_FLEXCOM_SPI_NCS);
  
#if !defined(CONFIG_SAMA5_FLEXCOM0_SPI)
  DEBUGASSERT(flexcom_spino != 0);
#endif
#if !defined(CONFIG_SAMA5_FLEXCOM1_SPI)
  DEBUGASSERT(flexcom_spino != 1);
#endif
#if !defined(CONFIG_SAMA5_FLEXCOM2_SPI)
  DEBUGASSERT(flexcom_spino != 2);
#endif
#if !defined(CONFIG_SAMA5_FLEXCOM3_SPI)
  DEBUGASSERT(flexcom_spino != 3);
#endif
#if !defined(CONFIG_SAMA5_FLEXCOM4_SPI)
  DEBUGASSERT(flexcom_spino != 4);
#endif

  /* Allocate a new state structure for this chip select.  NOTE that there
   * is no protection if the same chip select is used in two different
   * chip select structures.
   */
  flexcom_spics = (struct sam_flexcom_spics_s *)kmm_zalloc(sizeof(struct sam_flexcom_spics_s));
  if (!flexcom_spics)
    {
      spierr("ERROR: Failed to allocate a flexcom chip select structure\n");
      return NULL;
    }

  /* Set up the initial state for this chip select structure.  Other fields
   * were zeroed by kmm_zalloc().
   */

#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMA
  /* Can we do DMA on this peripheral? */

    /* TRH - not sure!! 
      Need to revisit once this is working withour DMA*/
  flexcom_spics->candma = flexcom_spino ? SAMA5_SPI1_DMA : SAMA5_SPI0_DMA;

  /* Pre-allocate DMA channels.  These allocations exploit that fact that
   * SPI0 is managed by DMAC0 and SPI1 is managed by DMAC1.  Hence,
   * the SPI number (spino) is the same as the DMAC number.
   */

  if (flexcom_spics->candma)
    {
      flexcom_spics->rxdma = sam_dmachannel(flexcom_spino, 0);
      if (!flexcom_spics->rxdma)
        {
          spierr("ERROR: Failed to allocate the RX DMA channel\n");
          flexcom_spics->candma = false;
        }
    }

  if (flexcom_spics->candma)
    {
      flexcom_spics->txdma = sam_dmachannel(flexcom_spino, 0);
      if (!flexcom_spics->txdma)
        {
          spierr("ERROR: Failed to allocate the TX DMA channel\n");
          sam_dmafree(flexcom_spics->rxdma);
          flexcom_spics->rxdma  = NULL;
          flexcom_spics->candma = false;
        }
    }
#endif

  /* Select the SPI operations */
  /* have already used DEBUGASSERT to weed out invalid SPI port assignments */
#if defined (CONFIG_SAMA5_FLEXCOM0_SPI)
  if (flexcom_spino == 0)
  {
    putreg32(FLEX_MR_OPMODE_SPI, SAM_FLEXCOM0_MR);
    flexcom_spics->flexcom_spidev.ops = &g_flexcom0_spiops;
    //save spi number and cs
    flexcom_spics->cs = flexcom_csno;
    flexcom_spics->flexcom_spino=flexcom_spino;
  }
#endif
#if defined (CONFIG_SAMA5_FLEXCOM1_SPI)
  if (flexcom_spino == 1)
  {
    putreg32(FLEX_MR_OPMODE_SPI, SAM_FLEXCOM1_MR);
    flexcom_spics->flexcom_spidev.ops = &g_flexcom1_spiops;
        //save spi number and cs
    flexcom_spics->cs = flexcom_csno;
    flexcom_spics->flexcom_spino=flexcom_spino;
  }
#endif
#if defined (CONFIG_SAMA5_FLEXCOM2_SPI)
  if (flexcom_spino == 2)
  {
    putreg32(FLEX_MR_OPMODE_SPI, SAM_FLEXCOM2_MR);
    flexcom_spics->flexcom_spidev.ops = &g_flexcom2_spiops;
    //save spi number and cs
    flexcom_spics->cs = flexcom_csno;
    flexcom_spics->flexcom_spino=flexcom_spino;
  }
#endif
#if defined (CONFIG_SAMA5_FLEXCOM3_SPI)
  if (flexcom_spino == 3)
  {
    putreg32(FLEX_MR_OPMODE_SPI, SAM_FLEXCOM3_MR);
    flexcom_spics->flexcom_spidev.ops = &g_flexcom3_spiops;
    //save spi number and cs
    flexcom_spics->cs = flexcom_csno;
    flexcom_spics->flexcom_spino=flexcom_spino;    
  }
#endif
#if defined (CONFIG_SAMA5_FLEXCOM4_SPI)
  if (flexcom_spino == 4)
  {
    putreg32(FLEX_MR_OPMODE_SPI, SAM_FLEXCOM4_MR);
    flexcom_spics->flexcom_spidev.ops = &g_flexcom4_spiops;
    //save spi number and cs
    flexcom_spics->cs = flexcom_csno;
    flexcom_spics->flexcom_spino=flexcom_spino;  
  }
#endif

  /* Get the SPI device structure associated with the chip select */

  flexcom_spi = flexcom_spi_device(flexcom_spics);

  /* Has the SPI hardware been initialized? */

  if (!flexcom_spi->initialized)
    {
      /* Enable clocking to the flexcom SPI block(s) */

      flags = enter_critical_section();
#if defined(CONFIG_SAMA5_FLEXCOM0_SPI)
      if (flexcom_spino == 0)
        {
          sam_flexcom0_enableclk();

          /* Configure multiplexed pins as connected on the board,
           * by board-specific logic.
          */
        }
#endif
#if defined(CONFIG_SAMA5_FLEXCOM1_SPI)
      if (flexcom_spino == 1)
        {
          sam_flexcom1_enableclk();

          /* Configure multiplexed pins as connected on the board,
           * by board-specific logic.
          */
        }
#endif
#if defined(CONFIG_SAMA5_FLEXCOM2_SPI)
      if (flexcom_spino == 2)
        {
          sam_flexcom2_enableclk();

          /* Configure multiplexed pins as connected on the board,
           * by board-specific logic.
          */
        }
#endif
#if defined(CONFIG_SAMA5_FLEXCOM3_SPI)
      if (flexcom_spino == 3)
        {
          sam_flexcom3_enableclk();

          /* Configure multiplexed pins as connected on the board,
           * by board-specific logic.
          */
        }
#endif
#if defined(CONFIG_SAMA5_FLEXCOM4_SPI)
      if (flexcom_spino == 4)
        {
          sam_flexcom4_enableclk();

          /* Configure multiplexed pins as connected on the board,
           * by board-specific logic.
          */
        }
#endif

#if  defined (CONFIG_SAMA5_FLEXCOM0_SPI) || defined (CONFIG_SAMA5_FLEXCOM1_SPI)\
  || defined (CONFIG_SAMA5_FLEXCOM2_SPI) || defined (CONFIG_SAMA5_FLEXCOM3_SPI)\
  || defined (CONFIG_SAMA5_FLEXCOM4_SPI)
      /* Disable SPI clocking */

      flexcom_spi_putreg(flexcom_spi, FLEXCOM_SPI_CR_SPIDIS, SAM_FLEXCOM_SPI_CR_OFFSET);

      /* Execute a software reset of the SPI (twice) */

      flexcom_spi_putreg(flexcom_spi, FLEXCOM_SPI_CR_SWRST, SAM_FLEXCOM_SPI_CR_OFFSET);
      flexcom_spi_putreg(flexcom_spi, FLEXCOM_SPI_CR_SWRST, SAM_FLEXCOM_SPI_CR_OFFSET);
      
      leave_critical_section(flags);

      /* Configure the SPI mode register */

      flexcom_spi_putreg(flexcom_spi, FLEXCOM_SPI_MR_MSTR | FLEXCOM_SPI_MR_MODFDIS, SAM_FLEXCOM_SPI_MR_OFFSET);

      /* And enable the SPI */

      flexcom_spi_putreg(flexcom_spi, FLEXCOM_SPI_CR_SPIEN, SAM_FLEXCOM_SPI_CR_OFFSET);
      up_mdelay(20);

      /* Flush any pending transfers */

      flexcom_spi_getreg(flexcom_spi, SAM_FLEXCOM_SPI_SR_OFFSET);
      flexcom_spi_getreg(flexcom_spi, SAM_FLEXCOM_SPI_RDR_OFFSET);

      /* Initialize the SPI semaphore that enforces mutually exclusive
       * access to the SPI registers.
       */

      nxsem_init(&flexcom_spi->flexcom_spisem, 0, 1);
      flexcom_spi->initialized = true;    


#ifdef CONFIG_SAMA5_FLEXCOM_SPI_DMA
      /* Initialize the SPI semaphore that is used to wake up the waiting
       * thread when the DMA transfer completes.  This semaphore is used for
       * signaling and, hence, should not have priority inheritance enabled.
       */

      nxsem_init(&flexcom_spics->dmawait, 0, 0);
      nxsem_set_protocol(&flexcom_spics->dmawait, SEM_PRIO_NONE);
#endif

      flexcom_spi_dumpregs(flexcom_spi, "After initialization");
    }

  /* Set to mode=0 and nbits=8 and impossible frequency. The SPI will only
   * be reconfigured if there is a change.
   */

  offset = (unsigned int)g_csroffset[flexcom_csno];
  regval = flexcom_spi_getreg(flexcom_spi, offset);
  regval &= ~(FLEXCOM_SPI_CSR_CPOL | FLEXCOM_SPI_CSR_NCPHA | FLEXCOM_SPI_CSR_BITS_MASK);
  regval |= (FLEXCOM_SPI_CSR_NCPHA | FLEXCOM_SPI_CSR_BITS(8));
  flexcom_spi_putreg(flexcom_spi, regval, offset);

  flexcom_spics->nbits = 8;
  spiinfo("csr[offset=%02x]=%08" PRIx32 "\n", offset, regval);

  return &flexcom_spics->flexcom_spidev;
}
#endif /* any FLEXCOM_SPI defined */

#endif
