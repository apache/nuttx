/****************************************************************************
 * arch/arm/src/lpc2378/lpc23xx_spi.c
 *
 *   Copyright (C) 2013 Li Zhuoyi. All rights reserved.
 *   Author: Li Zhuoyi <lzyy.cn@gmail.com>
 *
 * Derived from  arch/arm/src/lpc17xx_40xx/lpc17_40_spi.c
 *
 *   Copyright (C) 2010-2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"
#include "lpc23xx_pinsel.h"
#include "lpc23xx_scb.h"
#include "lpc23xx_spi.h"

#ifdef CONFIG_LPC2378_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
#  error CONFIG_SPI_EXCHANGE is not supported by this driver
#endif

/* SPI Clocking.
 *
 * The CPU clock is divided by by 1, 2, 4, or 8 to get the SPI peripheral
 * clock (SPI_CLOCK).  SPI_CLOCK may be further divided by 8-254 to get the
 * SPI clock.  If we want a usable range of 4KHz to 25MHz for the SPI, then:
 *
 * 1. SPICLK must be greater than (8*25MHz) = 200MHz (so we can't reach
 *    25MHz), and
 * 2. SPICLK must be less than (254*40Khz) = 101.6MHz.
 *
 * If we assume that CCLK less than or equal to 100MHz, we can just
 * use the CCLK undivided to get the SPI_CLOCK.
 *
 * NOTE: CCLK must be defined in the board.h header file.
 */

#define SPI_PCLKSET_DIV    SYSCON_PCLKSEL_CCLK
#define SPI_CLOCK          CCLK

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the SSP driver */

struct lpc23xx_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  mutex_t          lock;       /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of word in bits (8 to 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI methods */

static int      spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency);
static void     spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);
static void     spi_sndblock(struct spi_dev_s *dev,
                             const void *buffer, size_t nwords);
static void     spi_recvblock(struct spi_dev_s *dev, void *buffer,
                              size_t nwords);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spiops =
{
  .lock              = spi_lock,
  .select            = lpc23xx_spiselect,   /* Provided externally */
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                   /* Not supported */
#endif
  .status            = lpc23xx_spistatus,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc23xx_spicmddata,
#endif
  .send              = spi_send,
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc23xx_spiregister, /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

static struct lpc23xx_spidev_s g_spidev =
{
  .spidev            =
    {
      &g_spiops
    },
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
  struct lpc23xx_spidev_s *priv = (struct lpc23xx_spidev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
    }

  return ret;
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

static uint32_t spi_setfrequency(struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  struct lpc23xx_spidev_s *priv = (struct lpc23xx_spidev_s *)dev;
  uint32_t divisor;
  uint32_t actual;

  DEBUGASSERT(frequency <= SPI_CLOCK / 2);

  /* Check if the requested frequency is the same as the frequency
   * selection.
   */

  DEBUGASSERT(priv != NULL);
  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* frequency = SPI_CLOCK / divisor, or divisor = SPI_CLOCK / frequency */

  divisor = SPI_CLOCK / frequency;

  /* The SPI CCR register must contain an even number greater than or equal
   * to 8.
   */

  if (divisor < 8)
    {
      divisor = 8;
    }
  else if (divisor > 254)
    {
      divisor = 254;
    }

  divisor = (divisor + 1) & ~1;

  /* Save the new divisor value */

  putreg32(divisor, SPI_CCR);

  /* Calculate the new actual */

  actual = SPI_CLOCK / divisor;

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = actual;

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
  struct lpc23xx_spidev_s *priv = (struct lpc23xx_spidev_s *)dev;
  uint32_t regval;

  /* Has the mode changed? */

  DEBUGASSERT(priv != NULL);
  if (mode != priv->mode)
    {
      /* Yes... Set CR appropriately */

      regval = getreg32(SPI_CR);
      regval &= ~(SPI_CR_CPOL | SPI_CR_CPHA);

      switch (mode)
        {
          case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
            break;

          case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
            regval |= SPI_CR_CPHA;
            break;

          case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
            regval |= SPI_CR_CPOL;
            break;

          case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
            regval |= (SPI_CR_CPOL | SPI_CR_CPHA);
            break;

          default:
            DEBUGASSERT(FALSE);
            return;
        }

      putreg32(regval, SPI_CR);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
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
  struct lpc23xx_spidev_s *priv = (struct lpc23xx_spidev_s *)dev;
  uint32_t regval;

  DEBUGASSERT(nbits > 7 && nbits < 17);

  /* Has the number of bits changed? */

  DEBUGASSERT(priv != NULL);
  if (nbits != priv->nbits)
    {
      /* Yes... Set CR appropriately */

      regval  = getreg32(SPI_CR);
      regval &= ~SPI_CR_BITS_MASK;
      regval |= (nbits << SPI_CR_BITS_SHIFT) & SPI_CR_BITS_MASK;
      regval |= SPI_CR_BITENABLE;
      putreg32(regval, SPI_CR);

      /* Save the selection so that subsequent re-configurations will be
       * faster.
       */

      priv->nbits = nbits;
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
  /* Write the data to transmitted to the SPI Data Register */

  putreg32(wd, SPI_DR);

  /* Wait for the SPIF bit in the SPI Status Register to be set to 1. The
   * SPIF bit will be set after the last sampling clock edge of the SPI
   * data transfer.
   */

  while ((getreg32(SPI_SR) & SPI_SR_SPIF) == 0);

  /* Read the SPI Status Register again to clear the status bit */

  getreg32(SPI_SR);
  return getreg32(SPI_DR);
}

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

static void spi_sndblock(struct spi_dev_s *dev,
                         const void *buffer, size_t nwords)
{
  uint8_t *ptr = (uint8_t *)buffer;
  uint8_t data;

  spiinfo("nwords: %d\n", nwords);
  while (nwords)
    {
      /* Write the data to transmitted to the SPI Data Register */

      data = *ptr++;
      putreg32((uint32_t)data, SPI_DR);

      /* Wait for the SPIF bit in the SPI Status Register to be set to 1. The
       * SPIF bit will be set after the last sampling clock edge of the SPI
       * data transfer.
       */

      while ((getreg32(SPI_SR) & SPI_SR_SPIF) == 0);

      /* Read the SPI Status Register again to clear the status bit */

      getreg32(SPI_SR);
      nwords--;
    }
}

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

static void spi_recvblock(struct spi_dev_s *dev, void *buffer,
                          size_t nwords)
{
  uint8_t *ptr = (uint8_t *)buffer;

  spiinfo("nwords: %d\n", nwords);
  while (nwords)
    {
      /* Write some dummy data to the SPI Data Register in order to clock the
       * read data.
       */

      putreg32(0xff, SPI_DR);

      /* Wait for the SPIF bit in the SPI Status Register to be set to 1. The
       * SPIF bit will be set after the last sampling clock edge of the SPI
       * data transfer.
       */

      while ((getreg32(SPI_SR) & SPI_SR_SPIF) == 0);

      /* Read the SPI Status Register again to clear the status bit */

      getreg32(SPI_SR);

      /* Read the received data from the SPI Data Register */

      *ptr++ = (uint8_t)getreg32(SPI_DR);
      nwords--;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc23_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *lpc23_spibus_initialize(int port)
{
  struct lpc23xx_spidev_s *priv = &g_spidev;
  irqstate_t flags;
  uint32_t regval;

  /* Configure multiplexed pins as connected on the board.  Chip select
   * pins must be configured by board-specific logic.  All SPI pins and
   * one SPI1 pin (SCK) have multiple, alternative pin selection.
   * Definitions in the board.h file must be provided to resolve the
   * board-specific pin configuration like:
   *
   * #define GPIO_SPI_SCK GPIO_SPI_SCK_1
   */

  flags = enter_critical_section();

  regval  = getreg32(LPC23XX_SCB_BASE + SCB_PCONP_OFFSET);
  regval |= PCSPI;
  putreg32(regval, LPC23XX_SCB_BASE + SCB_PCONP_OFFSET);

  regval  = getreg32(LPC23XX_SCB_BASE + SCB_PCLKSEL0_OFFSET);
  regval &= ~SPI_PCLKSEL_MASK;
  regval |= SPI_PCLKSEL;
  putreg32(regval, LPC23XX_SCB_BASE + SCB_PCLKSEL0_OFFSET);

  regval  = getreg32(LPC23XX_PINSEL0);
  regval &= ~SPI_PINSEL0_MASK ;
  regval |= SPI_PINSEL0;
  putreg32(regval, LPC23XX_PINSEL0);

  regval  = getreg32(LPC23XX_PINSEL1);
  regval &= ~SPI_PINSEL1_MASK ;
  regval |= SPI_PINSEL1;
  putreg32(regval, LPC23XX_PINSEL1);

  leave_critical_section(flags);

  /* Configure 8-bit SPI mode and master mode */

  putreg32(SPI_CR_BITS_8BITS | SPI_CR_BITENABLE | SPI_CR_MSTR, SPI_CR);

  /* Set the initial SPI configuration */

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Initialize the SPI mutex that enforces mutually exclusive access */

  nxmutex_init(&priv->lock);
  return &priv->spidev;
}

#endif /* CONFIG_LPC2378_SPI */
