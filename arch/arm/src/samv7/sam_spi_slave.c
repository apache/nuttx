/****************************************************************************
 * arch/arm/src/samv7/sam_spi_slave.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spi/slave.h>

#include "up_arch.h"

#include "sam_config.h"
#include "sam_gpio.h"
#include "sam_periphclks.h"
#include "sam_spi.h"

#include "chip/sam_spi.h"
#include "chip/sam_pinmap.h"
#include <arch/board/board.h>

#ifdef CONFIG_SAMV7_SPI_SLAVE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Debug *******************************************************************/
/* Check if SPI debug is enabled (non-standard.. no support in
 * include/debug.h
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_SPI
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spidbg lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of the one SPI chip select */

struct sam_spics_s
{
  struct spi_sctrlr_s sctrlr;  /* Externally visible part of the SPI slave
                                * controller interface */
  FAR struct spi_sdev_s *sdev; /* Bound SPI slave device interface */

  uint8_t mode;                /* Mode 0,1,2,3 */
  uint8_t nbits;               /* Width of word in bits (8 to 16) */
  uint8_t spino;               /* SPI controller number (0 or 1) */
  uint8_t cs;                  /* Chip select number */
};

/* The overall state of one SPI controller */

struct sam_spidev_s
{
  uint32_t base;               /* SPI controller register base address */
  sem_t spisem;                /* Assures mutually exclusive access to SPI */
  bool initialized;            /* TRUE: Controller has been initialized */

  /* Debug stuff */

#ifdef CONFIG_SAMV7_SPI_REGDEBUG
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

#ifdef CONFIG_SAMV7_SPI_REGDEBUG
static bool     spi_checkreg(struct sam_spidev_s *spidev, bool wr,
                  uint32_t value, uint32_t address);
#else
# define        spi_checkreg(spidev,wr,value,address) (false)
#endif

static inline uint32_t spi_getreg(struct sam_spidev_s *spidev,
                  unsigned int offset);
static inline void spi_putreg(struct sam_spidev_s *spidev, uint32_t value,
                  unsigned int offset);
static inline struct sam_spidev_s *spi_device(struct sam_spics_s *spics);

#if defined(CONFIG_DEBUG_SPI) && defined(CONFIG_DEBUG_VERBOSE)
static void     spi_dumpregs(struct sam_spidev_s *spidev, const char *msg);
#else
# define        spi_dumpregs(spidev,msg)
#endif

static void     spi_semtake(struct sam_spidev_s *spidev);
#define         spi_semgive(spidev) (sem_post(&(spidev)->spisem))

static inline void spi_flush(struct sam_spidev_s *spidev);
static inline uint32_t spi_cs2pcs(struct sam_spics_s *spics);
static void     spi_setmode(FAR struct sam_spics_s *spics,
                  FAR struct sam_spidev_s *spidev, enum spi_smode_e mode);
static void     spi_setbits(FAR struct sam_spics_s *spics,
                  FAR struct sam_spidev_s *spidev, int nbits);

/* SPI slave controller methods */

static void     spi_bind(FAR struct spi_sctrlr_s *sctrlr, 
                  FAR struct spi_sdev_s *sdev, enum spi_smode_e mode,
                  int nbits);
static void     spi_unbind(FAR struct spi_sctrlr_s *sctrlr);
static void     spi_setdata(FAR struct spi_sctrlr_s *sctrlr, uint16_t data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array maps chip select numbers (0-3) to CSR register offsets */

static const uint8_t g_csroffset[4] =
{
  SAM_SPI_CSR0_OFFSET, SAM_SPI_CSR1_OFFSET,
  SAM_SPI_CSR2_OFFSET, SAM_SPI_CSR3_OFFSET
};

/* SPI slave controller driver operations */

static const struct spi_sctrlrops_s g_sctrlr_ops =
{
  .bind              = spi_bind,
  .unbind            = spi_unbind,
  .setdata           = spi_setdata,
};

#ifdef CONFIG_SAMV7_SPI0_SLAVE

/* This is the overall state of the SPI0 controller */

static struct sam_spidev_s g_spi0_sctrlr;
#endif

#ifdef CONFIG_SAMV7_SPI1_SLAVE
/* This is the overall state of the SPI0 controller */

static struct sam_spidev_s g_spi1_sctrlr;
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

#ifdef CONFIG_SAMV7_SPI_REGDEBUG
static bool spi_checkreg(struct sam_spidev_s *spidev, bool wr, uint32_t value,
                         uint32_t address)
{
  if (wr      == spidev->wrlast &&     /* Same kind of access? */
      value   == spidev->valuelast &&  /* Same value? */
      address == spidev->addresslast)  /* Same address? */
    {
      /* Yes, then just keep a count of the number of times we did this. */

      spidev->ntimes++;
      return false;
    }
  else
    {
      /* Did we do the previous operation more than once? */

      if (spidev->ntimes > 0)
        {
          /* Yes... show how many times we did it */

          lldbg("...[Repeats %d times]...\n", spidev->ntimes);
        }

      /* Save information about the new access */

      spidev->wrlast      = wr;
      spidev->valuelast   = value;
      spidev->addresslast = address;
      spidev->ntimes      = 0;
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

static inline uint32_t spi_getreg(struct sam_spidev_s *spidev,
                                  unsigned int offset)
{
  uint32_t address = spidev->base + offset;
  uint32_t value = getreg32(address);

#ifdef CONFIG_SAMV7_SPI_REGDEBUG
  if (spi_checkreg(spidev, false, value, address))
    {
      lldbg("%08x->%08x\n", address, value);
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

static inline void spi_putreg(struct sam_spidev_s *spidev, uint32_t value,
                              unsigned int offset)
{
  uint32_t address = spidev->base + offset;

#ifdef CONFIG_SAMV7_SPI_REGDEBUG
  if (spi_checkreg(spidev, true, value, address))
    {
      lldbg("%08x<-%08x\n", address, value);
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
 *   spidev - The SPI controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_DEBUG_SPI) && defined(CONFIG_DEBUG_VERBOSE)
static void spi_dumpregs(struct sam_spidev_s *spidev, const char *msg)
{
  spivdbg("%s:\n", msg);
  spivdbg("    MR:%08x   SR:%08x  IMR:%08x\n",
          getreg32(spidev->base + SAM_SPI_MR_OFFSET),
          getreg32(spidev->base + SAM_SPI_SR_OFFSET),
          getreg32(spidev->base + SAM_SPI_IMR_OFFSET));
  spivdbg("  CSR0:%08x CSR1:%08x CSR2:%08x CSR3:%08x\n",
          getreg32(spidev->base + SAM_SPI_CSR0_OFFSET),
          getreg32(spidev->base + SAM_SPI_CSR1_OFFSET),
          getreg32(spidev->base + SAM_SPI_CSR2_OFFSET),
          getreg32(spidev->base + SAM_SPI_CSR3_OFFSET));
  spivdbg("  WPCR:%08x WPSR:%08x\n",
          getreg32(spidev->base + SAM_SPI_WPCR_OFFSET),
          getreg32(spidev->base + SAM_SPI_WPSR_OFFSET));
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
#if defined(CONFIG_SAMV7_SPI0_SLAVE) && defined(CONFIG_SAMV7_SPI1_SLAVE)
  return spics->spino ? &g_spi1_sctrlr : &g_spi0_sctrlr;
#elif defined(CONFIG_SAMV7_SPI0_SLAVE)
  return &g_spi0_sctrlr;
#else
  return &g_spi1_sctrlr;
#endif
}

/****************************************************************************
 * Name: spi_semtake
 *
 * Description:
 *   Take the semaphore that enforces mutually exclusive access to SPI
 *   resources, handling any exceptional conditions
 *
 * Input Parameters:
 *   spidev - A reference to the MCAN peripheral state
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void spi_semtake(struct sam_spidev_s *spidev)
{
  int ret;

  /* Wait until we successfully get the semaphore.  EINTR is the only
   * expected 'failure' (meaning that the wait for the semaphore was
   * interrupted by a signal.
   */

  do
    {
      ret = sem_wait(&spidev->spisem);
      DEBUGASSERT(ret == 0 || errno == EINTR);
    }
  while (ret < 0);
}

/****************************************************************************
 * Name: spi_flush
 *
 * Description:
 *   Make sure that there are now dangling SPI transfer in progress
 *
 * Input Parameters:
 *   spidev - SPI controller state
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_flush(struct sam_spidev_s *spidev)
{
  /* Make sure the no TX activity is in progress... waiting if necessary */

  while ((spi_getreg(spidev, SAM_SPI_SR_OFFSET) & SPI_INT_TXEMPTY) == 0);

  /* Then make sure that there is no pending RX data .. reading as
   * discarding as necessary.
   */

  while ((spi_getreg(spidev, SAM_SPI_SR_OFFSET) & SPI_INT_RDRF) != 0)
    {
       (void)spi_getreg(spidev, SAM_SPI_RDR_OFFSET);
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
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode. See enum spi_smode_e for mode definitions
 *
 * Input Parameters:
 *   spics  - Chip select data structure
 *   spidev - SPI device data structure
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setmode(FAR struct sam_spics_s *spics,
                        FAR struct sam_spidev_s *spidev, enum spi_smode_e mode)
{
  uint32_t regval;
  unsigned int offset;

  spivdbg("cs=%d mode=%d\n", spics->cs, mode);

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

      offset = (unsigned int)g_csroffset[spics->cs];
      regval  = spi_getreg(spidev, offset);
      regval &= ~(SPI_CSR_CPOL | SPI_CSR_NCPHA);

      switch (mode)
        {
        case SPISLAVE_MODE0: /* CPOL=0; NCPHA=1 */
          regval |= SPI_CSR_NCPHA;
          break;

        case SPISLAVE_MODE1: /* CPOL=0; NCPHA=0 */
          break;

        case SPISLAVE_MODE2: /* CPOL=1; NCPHA=1 */
          regval |= (SPI_CSR_CPOL | SPI_CSR_NCPHA);
          break;

        case SPISLAVE_MODE3: /* CPOL=1; NCPHA=0 */
          regval |= SPI_CSR_CPOL;
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

      spi_putreg(spidev, regval, offset);
      spivdbg("csr[offset=%02x]=%08x\n", offset, regval);

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
 *   spics  - Chip select data structure
 *   spidev - SPI device data structure
 *   nbits  - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setbits(FAR struct sam_spics_s *spics,
                        FAR struct sam_spidev_s *spidev, int nbits)
{
  uint32_t regval;
  unsigned int offset;

  spivdbg("cs=%d nbits=%d\n", spics->cs, nbits);
  DEBUGASSERT(spics && nbits > 7 && nbits < 17);

  /* Has the number of bits changed? */

  if (nbits != spics->nbits)
    {
      /* Yes... Set number of bits appropriately */

      offset  = (unsigned int)g_csroffset[spics->cs];
      regval  = spi_getreg(spidev, offset);
      regval &= ~SPI_CSR_BITS_MASK;
      regval |= SPI_CSR_BITS(nbits);
      spi_putreg(spidev, regval, offset);

      spivdbg("csr[offset=%02x]=%08x\n", offset, regval);

      /* Save the selection so the subsequence re-configurations will be faster */

      spics->nbits = nbits;
    }
}

/****************************************************************************
 * Name: spi_bind
 *
 * Description:
 *   Bind the SPI slave device interface to the SPI slave controller
 *   interface and configure the SPI interface.  Upon return, the SPI
 *   slave controller driver is fully operational and ready to perform
 *   transfers.
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *   sdev   - SPI slave device interface instance
 *   mode   - The SPI mode requested
 *   nbits  - The number of bits requests.
 *            If value is greater > 0 then it implies MSB first
 *            If value is below < 0, then it implies LSB first with -nbits
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_bind(FAR struct spi_sctrlr_s *sctrlr,
                     FAR struct spi_sdev_s *sdev, enum spi_smode_e mode,
                     int nbits)
{
  FAR struct sam_spics_s *spics = (FAR struct sam_spics_s *)sctrlr;
  FAR struct sam_spidev_s *spidev;

  spivdbg("sdev=%p mode=%d nbits=%d\n", sdv, mode, nbits);

  DEBUGASSERT(spics != NULL && spics->sdev == NULL);
  spidev = spi_device(spics);

  /* Get exclusive access to the SPI device */

  spi_semtake(spidev);

  /* Bind the SPI slave device interface instance to the SPI slave
   * controller interface.
   */

  spics->sdev = sdev;

  /* Setup to begin normal SPI operation */

  spi_setmode(spics, spidev, mode);
  spi_setbits(spics, spidev, nbits);

#warning Missing Logic
  spi_semgive(spidev);
}

/****************************************************************************
 * Name: spi_unbind
 *
 * Description:
 *   Un-bind the SPI slave device interface from the SPI slave controller
 *   interface.  Reset the SPI interface and restore the SPI slave
 *   controller driver to its initial state,
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_unbind(FAR struct spi_sctrlr_s *sctrlr)
{
  FAR struct sam_spics_s *spics = (FAR struct sam_spics_s *)sctrlr;
  FAR struct sam_spidev_s *spidev;

  DEBUGASSERT(spics != NULL);
  spivdbg("Unbinding %p\n", spics->sdev);

  DEBUGASSERT(spics->sdev != NULL);
  spidev = spi_device(spics);

  /* Get exclusive access to the SPI device */

  spi_semtake(spidev);

  /* Unbind the SPI slave interface */

  /* Reset and disabled the SPI device */
#warning Missing Logic
  spi_semgive(spidev);
}

/****************************************************************************
 * Name: spi_setdata
 *
 * Description:
 *   Set the next value to be shifted out from the interface.  This primes
 *   the controller driver for the next transfer but has no effect on any
 *   in-process or currently "committed" transfers
 *
 * Input Parameters:
 *   sctrlr - SPI slave controller interface instance
 *   data   - Command/data mode data value to be shifted out.  The width of
 *            the data must be the same as the nbits parameter previously
 *            provided to the bind() methods.
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void spi_setdata(FAR struct spi_sctrlr_s *sctrlr, uint16_t data)
{
  FAR struct sam_spics_s *spics = (FAR struct sam_spics_s *)sctrlr;
  FAR struct sam_spidev_s *spidev;

  spivdbg("data=%04x\n", data);

  DEBUGASSERT(spics != NULL && spics->sdev != NULL);
  spidev = spi_device(spics);

  /* Get exclusive access to the SPI device */

  spi_semtake(spidev);

  /* Save this new word as the next word to shifted out.  The current word
   * written to the TX data registers is "committed" and will not be
   * overwritten.
   */

#warning Missing Logic
  spi_semgive(spidev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_spi_slave_initialize
 *
 * Description:
 *   Initialize the selected SPI port in slave mode.
 *
 * Input Parameter:
 *   cs - Chip select number (identifying the "logical" SPI port)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_sctrlr_s *up_spi_slave_initialize(int port)
{
  struct sam_spidev_s *spidev;
  struct sam_spics_s *spics;
  int csno  = (port & __SPI_CS_MASK) >> __SPI_CS_SHIFT;
  int spino = (port & __SPI_SPI_MASK) >> __SPI_SPI_SHIFT;
  irqstate_t flags;
  uint32_t regval;
  unsigned int offset;

  /* The support SAM parts have only a single SPI port */

  spivdbg("port: %d csno: %d spino: %d\n", port, csno, spino);
  DEBUGASSERT(csno >= 0 && csno <= SAM_SPI_NCS);

#if defined(CONFIG_SAMV7_SPI0_SLAVE) && defined(CONFIG_SAMV7_SPI1_SLAVE)
  DEBUGASSERT(spino >= 0 && spino <= 1);
#elif defined(CONFIG_SAMV7_SPI0_SLAVE)
  DEBUGASSERT(spino == 0);
#else
  DEBUGASSERT(spino == 1);
#endif

  /* Allocate a new state structure for this chip select.  NOTE that there
   * is no protection if the same chip select is used in two different
   * chip select structures.
   */

  spics = (struct sam_spics_s *)zalloc(sizeof(struct sam_spics_s));
  if (!spics)
    {
      spidbg("ERROR: Failed to allocate a chip select structure\n");
      return NULL;
    }

  /* Set up the initial state for this chip select structure.  Other fields
   * were zeroed by zalloc().
   */

   /* Initialize the SPI operations */

  spics->sctrlr.ops = &g_sctrlr_ops;

  /* Save the chip select and SPI controller numbers */

  spics->cs        = csno;
  spics->spino     = spino;

  /* Get the SPI device structure associated with the chip select */

  spidev = spi_device(spics);

  /* Has the SPI hardware been initialized? */

  if (!spidev->initialized)
    {
      /* Enable clocking to the SPI block */

      flags = irqsave();
#if defined(CONFIG_SAMV7_SPI0_SLAVE) && defined(CONFIG_SAMV7_SPI1_SLAVE)
      if (spino == 0)
#endif
#if defined(CONFIG_SAMV7_SPI0_SLAVE)
        {
          /* Set the SPI0 register base address */

          spidev->base = SAM_SPI0_BASE,

          /* Enable peripheral clocking to SPI0 */

          sam_spi0_enableclk();

          /* Configure multiplexed pins as connected on the board.  Chip
           * select pins must be selected by board-specific logic.
           */

          sam_configgpio(GPIO_SPI0_MISO);
          sam_configgpio(GPIO_SPI0_MOSI);
          sam_configgpio(GPIO_SPI0_SPCK);
        }
#endif
#if defined(CONFIG_SAMV7_SPI0_SLAVE) && defined(CONFIG_SAMV7_SPI1_SLAVE)
      else
#endif
#if defined(CONFIG_SAMV7_SPI1_SLAVE)
        {
          /* Set the SPI1 register base address */

          spidev->base = SAM_SPI1_BASE,

          /* Enable peripheral clocking to SPI1 */

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

      spi_putreg(spidev, SPI_CR_SPIDIS, SAM_SPI_CR_OFFSET);

      /* Execute a software reset of the SPI (twice) */

      spi_putreg(spidev, SPI_CR_SWRST, SAM_SPI_CR_OFFSET);
      spi_putreg(spidev, SPI_CR_SWRST, SAM_SPI_CR_OFFSET);
      irqrestore(flags);

      /* Configure the SPI mode register */

      spi_putreg(spidev, SPI_MR_SLAVE | SPI_MR_MODFDIS, SAM_SPI_MR_OFFSET);

      /* And enable the SPI */

      spi_putreg(spidev, SPI_CR_SPIEN, SAM_SPI_CR_OFFSET);
      up_mdelay(20);

      /* Flush any pending transfers */

      (void)spi_getreg(spidev, SAM_SPI_SR_OFFSET);
      (void)spi_getreg(spidev, SAM_SPI_RDR_OFFSET);

      /* Initialize the SPI semaphore that enforces mutually exclusive
       * access to the SPI registers.
       */

      sem_init(&spidev->spisem, 0, 1);
      spidev->initialized = true;

      spi_dumpregs(spidev, "After initialization");
    }

  /* Set to mode=0 and nbits=8 */

  offset = (unsigned int)g_csroffset[csno];
  regval = spi_getreg(spidev, offset);
  regval &= ~(SPI_CSR_CPOL | SPI_CSR_NCPHA | SPI_CSR_BITS_MASK);
  regval |= (SPI_CSR_NCPHA | SPI_CSR_BITS(8));
  spi_putreg(spidev, regval, offset);

  spics->nbits = 8;
  spivdbg("csr[offset=%02x]=%08x\n", offset, regval);

  return &spics->sctrlr;
}
#endif /* CONFIG_SAMV7_SPI_SLAVE */
