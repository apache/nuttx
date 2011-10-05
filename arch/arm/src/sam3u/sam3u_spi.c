/****************************************************************************
 * arch/arm/src/sam3u/sam3u_spi.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/spi.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "sam3u_internal.h"
#include "sam3u_pmc.h"
#include "sam3u_spi.h"

#ifdef CONFIG_SAM3U_SPI

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef SPI_DEBUG     /* Define to enable debug */
#undef SPI_VERBOSE   /* Define to enable verbose debug */

#ifdef SPI_DEBUG
#  define spidbg  lldbg
#  ifdef SPI_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef SPI_VERBOSE
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The state of one chip select */

#ifndef CONFIG_SPI_OWNBUS
struct sam3u_chipselect_s
{
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of word in bits (8 to 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
};
#endif

/* The overall state of the SPI interface */

struct sam3u_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
#ifndef CONFIG_SPI_OWNBUS
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  struct sam3u_chipselect_s csstate[4];
#endif
  uint8_t          cs;         /* Chip select number */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI methods */

#ifndef CONFIG_SPI_OWNBUS
static int      spi_lock(FAR struct spi_dev_s *dev, bool lock);
#endif
static void     spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid,
                  bool selected);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                  uint32_t frequency);
static void     spi_setmode(FAR struct spi_dev_s *dev,
                  enum spi_mode_e mode);
static void     spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t ch);
#ifdef CONFIG_SPI_EXCHANGE
static void     spi_exchange(FAR struct spi_dev_s *dev,
                   FAR const void *txbuffer, FAR void *rxbuffer, size_t nwords);
#else
static void     spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords);
static void     spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SPI driver operations */

static const struct spi_ops_s g_spiops =
{
#ifndef CONFIG_SPI_OWNBUS
  .lock              = spi_lock,
#endif
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = sam3u_spistatus,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = sam3u_spicmddata,
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

/* SPI device structure */

static struct sam3u_spidev_s g_spidev =
{
  .spidev            = { &g_spiops },
}; 

/* This array maps chip select numbers (0-3) to CSR register addresses */

static const uint32_t g_csraddr[4] =
{
  SAM3U_SPI_CSR0, SAM3U_SPI_CSR1, SAM3U_SPI_CSR2, SAM3U_SPI_CSR3
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI busses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the busses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
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

#ifndef CONFIG_SPI_OWNBUS
static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)dev;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      while (sem_wait(&priv->exclsem) != 0)
        {
          /* The only case that an error should occur here is if the wait was awakened
           * by a signal.
           */

          ASSERT(errno == EINTR);
        }
    }
  else
    {
      (void)sem_post(&priv->exclsem);
    }
  return OK;
}
#endif

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

 static void spi_select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
 {
  FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)dev;
  uint32_t regval;

  /* Are we selecting or de-selecting the device? */

  if (selected)
    {
      /* At this point, we expect no chip selected */

      DEBUGASSERT(priv->cs == 0xff);

      /* Get the chip select associated with this SPI device */

      priv->cs = sam3u_spiselect(devid);
      DEBUGASSERT(priv->cs >= 0 && priv->cs <= 3);

      /* Before writing the TDR, the PCS field in the SPI_MR register must be set
       * in order to select a slave.
       */

      regval = getreg32(SAM3U_SPI_MR);
      regval &= ~SPI_MR_PCS_MASK;
      regval |= (priv->cs << SPI_MR_PCS_SHIFT);
      putreg32(regval, SAM3U_SPI_MR);
    }
  else
    {
      /* At this point, we expect the chip to have already been selected */

#ifdef CONFIG_DEBUG
      int cs = sam3u_spiselect(devid);
      DEBUGASSERT(priv->cs == cs);
#endif

      /* Mark no chip selected */

      priv->cs = 0xff;
    }
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

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency)
{
  FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)dev;
  uint32_t actual;
  uint32_t scbr;
  uint32_t dlybs;
  uint32_t dlybct;
  uint32_t regval;
  uint32_t regaddr;

  DEBUGASSERT(priv->cs >= 0 && priv->cs <= 3);

  /* Check if the requested frequency is the same as the frequency selection */

#ifndef CONFIG_SPI_OWNBUS
  if (priv->csstate[priv->cs].frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->csstate[priv->cs].actual;
    }
#endif

  /* Configure SPI to a frequency as close as possible to the requested frequency.
   *
   *   SPCK frequency = MCK / SCBR, or SCBR = MCK / frequency
   */

  scbr = SAM3U_MCK_FREQUENCY / frequency;

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
  
  regaddr = g_csraddr[priv->cs];
  regval  = getreg32(regaddr);
  regval &= ~(SPI_CSR_SCBR_MASK|SPI_CSR_DLYBS_MASK|SPI_CSR_DLYBCT_MASK);
  regval |= scbr << SPI_CSR_SCBR_SHIFT;

  /* DLYBS: Delay Before SPCK.  This field defines the delay from NPCS valid to the
   * first valid SPCK transition. When DLYBS equals zero, the NPCS valid to SPCK
   * transition is 1/2 the SPCK clock period. Otherwise, the following equations
   * determine the delay:
   *
   *   Delay Before SPCK = DLYBS / MCK
   *
   * For a 2uS delay
   *
   *   DLYBS = MCK * 0.000002 = MCK / 500000
   */

  dlybs   = SAM3U_MCK_FREQUENCY / 500000;
  regval |= dlybs << SPI_CSR_DLYBS_SHIFT;

  /* DLYBCT: Delay Between Consecutive Transfers.  This field defines the delay
   * between two consecutive transfers with the same peripheral without removing
   * the chip select. The delay is always inserted after each transfer and
   * before removing the chip select if needed.
   *
   *  Delay Between Consecutive Transfers = (32 x DLYBCT) / MCK
   *
   * For a 5uS delay:
   *
   *  DLYBCT = MCK * 0.000005 / 32 = MCK / 200000 / 32
   */

  dlybct  = SAM3U_MCK_FREQUENCY / 200000 / 32;
  regval |= dlybct << SPI_CSR_DLYBCT_SHIFT;
  putreg32(regval, regaddr);

  /* Calculate the new actual frequency */

  actual = SAM3U_MCK_FREQUENCY / scbr;

  /* Save the frequency setting */

#ifndef CONFIG_SPI_OWNBUS
  priv->csstate[priv->cs].frequency = frequency;
  priv->csstate[priv->cs].actual    = actual;
#endif

  spidbg("Frequency %d->%d\n", frequency, actual);
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

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)dev;
  uint32_t regval;
  uint32_t regaddr;

  DEBUGASSERT(priv->cs >= 0 && priv->cs <= 3);

  /* Has the mode changed? */

#ifndef CONFIG_SPI_OWNBUS
  if (mode != priv->csstate[priv->cs].mode)
    {
#endif
      /* Yes... Set the mode appropriately */

      regaddr = g_csraddr[priv->cs];
      regval  = getreg32(regaddr);
      regval &= ~(SPI_CSR_CPOL|SPI_CSR_NCPHA);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; NCPHA=0 */
          break;
 
        case SPIDEV_MODE1: /* CPOL=0; NCPHA=1 */
            regval |= SPI_CSR_NCPHA;
          break;
 
        case SPIDEV_MODE2: /* CPOL=1; NCPHA=0 */
            regval |= SPI_CSR_CPOL;
          break;
 
        case SPIDEV_MODE3: /* CPOL=1; NCPHA=1 */
          regval |= (SPI_CSR_CPOL|SPI_CSR_NCPHA);
          break;
 
        default:
          DEBUGASSERT(FALSE);
          return;
        }

      putreg32(regval, regaddr);        

      /* Save the mode so that subsequent re-configurations will be faster */

#ifndef CONFIG_SPI_OWNBUS
      priv->csstate[priv->cs].mode = mode;
    }
#endif
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

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)dev;
  uint32_t regaddr;
  uint32_t regval;

  /* Has the number of bits changed? */

  DEBUGASSERT(priv && nbits > 7 && nbits < 17);
  DEBUGASSERT(priv->cs >= 0 && priv->cs <= 3);

#ifndef CONFIG_SPI_OWNBUS
  if (nbits != priv->csstate[priv->cs].nbits)
    {
#endif
      /* Yes... Set number of bits appropriately */

      regaddr = g_csraddr[priv->cs];
      regval  = getreg32(regaddr);
      regval &= ~SPI_CSR_BITS_MASK;
      regval |= SPI_CSR_BITS(nbits);
      putreg32(regval, regaddr);

      /* Save the selection so the subsequence re-configurations will be faster */

#ifndef CONFIG_SPI_OWNBUS
      priv->csstate[priv->cs].nbits = nbits;
    }
#endif
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

static uint16_t spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
#ifdef CONFIG_SPI_VARSELECT
  FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)dev;
  uint32_t tdr = (uint32_t)priv->cs << SPI_TDR_PCS_SHIFT;
#endif

  /* Wait for any previous data written to the TDR to be transferred to the
   * serializer.
   */

  while ((getreg32(SAM3U_SPI_SR) & SPI_INT_TDRE) == 0);

  /* Write the data to transmitted to the Transmit Data Register (TDR) */

#ifdef CONFIG_SPI_VARSELECT
  putreg32((uint32_t)wd | tdr | SPI_TDR_LASTXFER, SAM3U_SPI_TDR);
#else
  putreg32((uint32_t)wd, SAM3U_SPI_TDR);
#endif

  /* Wait for the read data to be available in the RDR */

  while ((getreg32(SAM3U_SPI_SR) & SPI_INT_RDRF) == 0);

  /* Return the received data */
 
  return (uint16_t)getreg32(SAM3U_SPI_RDR);
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exahange a block of data from SPI. Required.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to recieve data
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

#ifdef CONFIG_SPI_EXCHANGE
static void  spi_exchange(FAR struct spi_dev_s *dev,
                          FAR const void *txbuffer, FAR void *rxbuffer,
                          size_t nwords)
{
#ifdef CONFIG_SPI_VARSELECT
  FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)dev;
  uint32_t tdr = (uint32_t)priv->cs << SPI_TDR_PCS_SHIFT;
#endif
  FAR uint8_t *rxptr = (FAR uint8_t*)rxbuffer;
  FAR uint8_t *txptr = (FAR uint8_t*)txbuffer;
  uint8_t data;

  spidbg("nwords: %d\n", nwords);

  /* Loop, sending each word in the user-provied data buffer */

  for ( ; nwords > 0; nwords--)
    {
      /* Wait for any previous data written to the TDR to be transferred
       * to the serializer.
       */
      
      while ((getreg32(SAM3U_SPI_SR) & SPI_INT_TDRE) == 0);

      /* Get the data to send (0xff if there is no data source) */

      if (rxptr)
        {
          data = *txptr++;
        }
      else
        {
          data = 0xff;
        }

      /* Write the data to transmitted to the Transmit Data Register (TDR) */

#ifdef CONFIG_SPI_VARSELECT
      if (nwords == 1)
        {
          tdr |= SPI_TDR_LASTXFER;
        }
      putreg32((uint32_t)data | tdr, SAM3U_SPI_TDR);
#else
      putreg32((uint32_t)data, SAM3U_SPI_TDR);
#endif

      /* Wait for the read data to be available in the RDR */

      while ((getreg32(SAM3U_SPI_SR) & SPI_INT_RDRF) == 0);

      /* Read the received data from the SPI Data Register */   

      data = (uint8_t)getreg32(SAM3U_SPI_RDR);
      if (rxptr)
        {
          *txptr++ = data;
        }
    }
}
#endif

/***************************************************************************
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
 *            packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer, size_t nwords)
{
#ifdef CONFIG_SPI_VARSELECT
  FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)dev;
  uint32_t tdr = (uint32_t)priv->cs << SPI_TDR_PCS_SHIFT;
#endif
  FAR uint8_t *ptr = (FAR uint8_t*)buffer;
  uint8_t data;

  spidbg("nwords: %d\n", nwords);

  /* Loop, sending each word in the user-provied data buffer */

  for ( ; nwords > 0; nwords--)
    {
      /* Wait for any previous data written to the TDR to be transferred
       * to the serializer.
       */
      
      while ((getreg32(SAM3U_SPI_SR) & SPI_INT_TDRE) == 0);

      /* Write the data to transmitted to the Transmit Data Register (TDR) */

      data = *ptr++;
#ifdef CONFIG_SPI_VARSELECT
      if (nwords == 1)
        {
          tdr |= SPI_TDR_LASTXFER;
        }
      putreg32((uint32_t)data | tdr, SAM3U_SPI_TDR);
#else
      putreg32((uint32_t)data, SAM3U_SPI_TDR);
#endif
    }
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
 *   buffer - A pointer to the buffer in which to recieve data
 *   nwords - the length of data that can be received in the buffer in number
 *            of words.  The wordsize is determined by the number of bits-per-word
 *            selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer, size_t nwords)
{
#ifdef CONFIG_SPI_VARSELECT
  FAR struct sam3u_spidev_s *priv = (FAR struct sam3u_spidev_s *)dev;
  uint32_t tdr = (uint32_t)priv->cs << SPI_TDR_PCS_SHIFT;
#endif
  FAR uint8_t *ptr = (FAR uint8_t*)buffer;

  spidbg("nwords: %d\n", nwords);

  /* Loop, receiving each word */

  for ( ; nwords > 0; nwords--)
    {
      /* Wait for any previous data written to the TDR to be transferred
       * to the serializer.
       */
  
      while ((getreg32(SAM3U_SPI_SR) & SPI_INT_TDRE) == 0);
  
      /* Write the some dummy data the Transmit Data Register (TDR) in order
       * to clock the read data.
       */

#ifdef CONFIG_SPI_VARSELECT
      if (nwords == 1)
        {
          tdr |= SPI_TDR_LASTXFER;
        }
      putreg32(0xff | tdr, SAM3U_SPI_TDR);
#else
      putreg32(0xff, SAM3U_SPI_TDR);
#endif
      /* Wait for the read data to be available in the RDR */

      while ((getreg32(SAM3U_SPI_SR) & SPI_INT_RDRF) == 0);

      /* Read the received data from the SPI Data Register */   

      *ptr++ = (uint8_t)getreg32(SAM3U_SPI_RDR);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_spiinitialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameter:
 *   Port number (for hardware that has mutiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *up_spiinitialize(int port)
{
  FAR struct sam3u_spidev_s *priv = &g_spidev;
  irqstate_t flags;
  uint32_t regval;

  /* The SAM3U has only a single SPI port */

  DEBUGASSERT(port == 0);

  /* Set up the initial state */

  priv->cs = 0xff;

  /* Apply power to the SPI block */

  flags = irqsave();
  regval = getreg32(SAM3U_PMC_PCER);
  regval |= (1 << SAM3U_PID_SPI);
#ifdef CONFIG_SAM3U_SPIINTERRUPT
  regval |= (1 << SAM3U_IRQ_SPI);
#endif
  putreg32(regval, SAM3U_PMC_PCER);

  /* Configure multiplexed pins as connected on the board.  Chip select pins
   * must be configured by board-specific logic.
   */

  sam3u_configgpio(GPIO_SPI0_MISO);
  sam3u_configgpio(GPIO_SPI0_MOSI);
  sam3u_configgpio(GPIO_SPI0_SPCK);

  /* Disable SPI clocking */

  putreg32(SPI_CR_SPIDIS, SAM3U_SPI_CR);

  /* Execute a software reset of the SPI (twice) */

  putreg32(SPI_CR_SWRST, SAM3U_SPI_CR);
  putreg32(SPI_CR_SWRST, SAM3U_SPI_CR);
  irqrestore(flags);

  /* Configure the SPI mode register */

  putreg32(SPI_MR_MSTR | SPI_MR_MODFDIS, SAM3U_SPI_MR);

  /* And enable the SPI */

  putreg32(SPI_CR_SPIEN, SAM3U_SPI_CR);
  up_mdelay(20);

  /* Flush any pending transfers */

  (void)getreg32(SAM3U_SPI_SR);
  (void)getreg32(SAM3U_SPI_RDR);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

#ifndef CONFIG_SPI_OWNBUS
  sem_init(&priv->exclsem, 0, 1);
#endif
  return &priv->spidev;
}
#endif /* CONFIG_SAM3U_SPI */
