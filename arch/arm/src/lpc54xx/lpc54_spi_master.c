/****************************************************************************
 * arch/arm/src/lpc54xx/lpc54_spi.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip/lpc54_pinmux.h"
#include "chip/lpc54_syscon.h"
#include "chip/lpc54_flexcomm.h"
#include "chip/lpc54_spi.h"
#include "lpc54_config.h"
#include "lpc54_enableclk.h"
#include "lpc54_spi_master.h"

#ifdef HAVE_SPI_MASTER_DEVICE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure descibes the state of the SSP driver */

struct lpc54_spidev_s
{
  struct spi_dev_s dev;  /* Externally visible part of the SPI interface */
  uintptr_t base;        /* Base address of Flexcomm registers */
  sem_t exclsem;         /* Held while chip is selected for mutual exclusion */
  uint32_t fclock;       /* Flexcomm function clock frequency */
  uint32_t frequency;    /* Requested clock frequency */
  uint32_t actual;       /* Actual clock frequency */
  uint16_t irq;          /* Flexcomm IRQ number */
  uint8_t nbits;        /* Width of word in bits (8 to 16) */
  uint8_t mode;         /* Mode 0,1,2,3 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline bool lpc54_spi_16bitmode(FAR struct lpc54_spidev_s *priv);

/* SPI methods */

static int      lpc54_spi_lock(FAR struct spi_dev_s *dev, bool lock);
static void     lpc54_spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                  bool selected);
static uint32_t lpc54_spi_setfrequency(FAR struct spi_dev_s *dev,
                  uint32_t frequency);
static void     lpc54_spi_setmode(FAR struct spi_dev_s *dev,
                  enum spi_mode_e mode);
static void     lpc54_spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint16_t lpc54_spi_send(FAR struct spi_dev_s *dev, uint16_t ch);
#ifdef CONFIG_LPC54_SPI_MASTER_DMA
static void     lpc54_spi_exchange_nodma(FAR struct spi_dev_s *dev,
                   FAR const void *txbuffer, FAR void *rxbuffer,
                   size_t nwords)
#endif
static void     lpc54_spi_exchange(FAR struct spi_dev_s *dev,
                  FAR const void *txbuffer, FAR void *rxbuffer,
                  size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void     lpc54_spi_sndblock(FAR struct spi_dev_s *dev,
                  FAR const void *buffer, size_t nwords);
static void     lpc54_spi_recvblock(FAR struct spi_dev_s *dev,
                  FAR void *buffer, size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct spi_ops_s g_spi_ops =
{
  .lock              = lpc54_spi_lock,
  .select            = lpc54_spiselect,
  .setfrequency      = lpc54_spi_setfrequency,
  .setmode           = lpc54_spi_setmode,
  .setbits           = lpc54_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,                 /* Not supported */
#endif
  .status            = lpc54_spistatus,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = lpc54_spicmddata,
#endif
  .send              = lpc54_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = lpc54_spi_exchange,
#else
  .sndblock          = lpc54_spi_sndblock,
  .recvblock         = lpc54_spi_recvblock,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = lpc54_spiregister, /* Provided externally */
#else
  .registercallback  = 0,                 /* Not implemented */
#endif
};

#ifdef CONFIG_LPC54_I2C0_MASTER
static struct lpc54_spidev_s g_spi0_dev;
#endif
#ifdef CONFIG_LPC54_I2C1_MASTER
static struct lpc54_spidev_s g_spi1_dev;
#endif
#ifdef CONFIG_LPC54_I2C2_MASTER
static struct lpc54_spidev_s g_spi2_dev;
#endif
#ifdef CONFIG_LPC54_I2C3_MASTER
static struct lpc54_spidev_s g_spi3_dev;
#endif
#ifdef CONFIG_LPC54_I2C4_MASTER
static struct lpc54_spidev_s g_spi4_dev;
#endif
#ifdef CONFIG_LPC54_I2C5_MASTER
static struct lpc54_spidev_s g_spi5_dev;
#endif
#ifdef CONFIG_LPC54_I2C6_MASTER
static struct lpc54_spidev_s g_spi6_dev;
#endif
#ifdef CONFIG_LPC54_I2C7_MASTER
static struct lpc54_spidev_s g_spi7_dev;
#endif
#ifdef CONFIG_LPC54_I2C8_MASTER
static struct lpc54_spidev_s g_spi8_dev;
#endif
#ifdef CONFIG_LPC54_I2C9_MASTER
static struct lpc54_spidev_s g_spi9_dev;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_spi_putreg
 *
 * Description:
 *   Write a value to a register at the offset from the Flexcomm base.
 *
 ****************************************************************************/

static inline void lpc54_spi_putreg(struct lpc54_spidev_s *priv,
                                    unsigned int regoffset, uint32_t regval)
{
  putreg32(value, priv->base + regoffset);
}

/****************************************************************************
 * Name: lpc54_spi_gettreg
 *
 * Description:
 *   Read the content of a register at the offset from the Flexcomm base.
 *
 ****************************************************************************/

static inline void lpc54_spi_gettreg(struct lpc54_spidev_s *priv,
                                     unsigned int regoffset)
{
  return getreg32(priv->base + regoffset);
}

/****************************************************************************
 * Name: lpc54_spi_16bitmode
 *
 * Description:
 *   Check if the SPI is operating in > 8-bit mode (16-bit accesses)
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   true: 16-bit mode, false: 8-bit mode
 *
 ****************************************************************************/

static inline bool lpc54_spi_16bitmode(FAR struct lpc54_spidev_s *priv)
{
#warning Missing logic
  return false;
}

/****************************************************************************
 * Name: lpc54_spi_lock
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

static int lpc54_spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct lpc54_spidev_s *priv = (FAR struct lpc54_spidev_s *)dev;
  int ret;

  if (lock)
    {
      /* Take the semaphore (perhaps waiting) */

      do
        {
          ret = nxsem_wait(&priv->exclsem);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          DEBUGASSERT(ret == OK || ret == -EINTR);
        }
      while (ret == -EINTR);
    }
  else
    {
      (void)nxsem_post(&priv->exclsem);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: lpc54_spi_setfrequency
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

static uint32_t lpc54_spi_setfrequency(FAR struct spi_dev_s *dev,
                                       uint32_t frequency)
{
  FAR struct lpc54_spidev_s *priv = (FAR struct lpc54_spidev_s *)dev;
  uint32_t actual;

  /* Check if the requested frequence is the same as the frequency selection */

  DEBUGASSERT(priv && frequency <= priv->fclock / 2);

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* Set the new SPI frequency */
#warning Missing logic

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = actual;

  spiinfo("Frequency %d->%d\n", frequency, actual);
  return actual;
}

/****************************************************************************
 * Name: lpc54_spi_setmode
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

static void lpc54_spi_setmode(FAR struct spi_dev_s *dev,
                              enum spi_mode_e mode)
{
  FAR struct lpc54_spidev_s *priv = (FAR struct lpc54_spidev_s *)dev;
  uint32_t regval;

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set the new mode */
#warning Missing logic

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
#warning Missing logic
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
#warning Missing logic
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
#warning Missing logic
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
#warning Missing logic
          break;

        default:
          DEBUGPANIC();
          return;
        }

#warning Missing logic

      /* Save the mode so that subsequent re-configuratins will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: lpc54_spi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   dev   -  Device-specific state data
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void lpc54_spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct lpc54_spidev_s *priv = (FAR struct lpc54_spidev_s *)dev;
  uint32_t regval;

  /* Has the number of bits changed? */

  DEBUGASSERT(priv && nbits > 7 && nbits < 17);

  if (nbits != priv->nbits)
    {
      /* Yes... Set the number word width */
#warning Missing logic

      /* Save the selection so the subsequence re-configurations will be faster */

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: lpc54_spi_send
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

static uint16_t lpc54_spi_send(FAR struct spi_dev_s *dev, uint16_t wd)
{
  uint16_t ret;

  /* Write the data to transmitted to the SPI Data Register */
#warning Missing logic

  /* Read the SPI Status Register again to clear the status bit */
#warning Missing logic

  return ret;
}

/****************************************************************************
 * Name: lpc54_spi_exchange (no DMA).  aka lpc54_spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchaned in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_LPC54_SPI_MASTER_DMA
static void lpc54_spi_exchange(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer, FAR void *rxbuffer,
                               size_t nwords)
#else
static void lpc54_spi_exchange_nodma(FAR struct spi_dev_s *dev,
                                     FAR const void *txbuffer,
                                     FAR void *rxbuffer, size_t nwords)
#endif
{
  FAR struct lpc54_spidev_s *priv = (FAR struct lpc54_spidev_s *)dev;
  DEBUGASSERT(priv && priv->base);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n",
          txbuffer, rxbuffer, nwords);

  /* 8- or 16-bit mode? */

  if (lpc54_spi_16bitmode(priv))
    {
      /* 16-bit mode */

      const uint16_t *src  = (const uint16_t *)txbuffer;
            uint16_t *dest = (uint16_t *)rxbuffer;
            uint16_t  word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
            {
              word = 0xffff;
            }

          /* Exchange one word */

          word = spi_send(dev, word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
  else
    {
      /* 8-bit mode */

      const uint8_t *src  = (const uint8_t *)txbuffer;
            uint8_t *dest = (uint8_t *)rxbuffer;
            uint8_t  word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
            {
              word = 0xff;
            }

          /* Exchange one word */

          word = (uint8_t)spi_send(dev, (uint16_t)word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
}

/****************************************************************************
 * Name: lpc54_spi_exchange (no DMA).  aka lpc54_spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchaned in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_LPC54_SPI_MASTER_DMA
static void lpc54_spi_exchange(FAR struct spi_dev_s *dev,
                               FAR const void *txbuffer, FAR void *rxbuffer,
                               size_t nwords)
{
  /* If the transfer is small, then perform the exchange without using DMA. */
#warning Missing logic

  /* Otherwise, use DMA */
#warning Missing logic
}
#endif /* CONFIG_LPC54_SPI_MASTER_DMA */

/****************************************************************************
 * Name: lpc54_spi_sndblock
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

static void lpc54_spi_sndblock(FAR struct spi_dev_s *dev,
                               FAR const void *buffer, size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return lpc54_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: lpc54_spi_recvblock
 *
 * Description:
 *   Revice a block of data from SPI
 *
 * Input Parameters:
 *   dev -    Device-specific state data
 *   buffer - A pointer to the buffer in which to recieve data
 *   nwords - the length of data that can be received in the buffer in
 *            number of words.  The wordsize is determined by the number of
 *            bits-per-word selected for the SPI interface.  If nbits <= 8,
 *            the data is packed into uint8_t's; if nbits >8, the data is
 *            packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpc54_spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                                size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return lpc54_spi_exchange(dev, NULL, rxbuffer, nwords);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port
 *   0 - SPI0
 *   1 - SPI1
 *   ...
 *   9 - SSP9
 *
 * Input Parameter:
 *   port - SPI peripheral number.  0..9
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *lpc54_spibus_initialize(int port)
{
  struct lpc54_spidev_s *priv;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Configure the requestin SPI peripheral */
  /* NOTE:  The basic FLEXCOMM initialization was performed in
   * lpc54_lowputc.c.
   */

#ifdef CONFIG_LPC54_I2C0_MASTER
  if (port == 0)
    {
      /* Attach 12 MHz clock to FLEXCOMM0 */

      lpc54_flexcomm0_enableclk();

      /* Set FLEXCOMM0 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM0_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi0_dev;
      priv->base     = LPC54_FLEXCOMM0_BASE;
      priv->irqid    = LPC54_IRQ_FLEXCOMM0;
      priv->fclock   = BOARD_FLEXCOMM0_FCLK;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C0_SCK);
      lpc54_gpio_config(GPIO_I2C0_MOSI);
      lpc54_gpio_config(GPIO_I2C0_MISO);

      /* Set up the FLEXCOMM0 function clock */

      putreg32(BOARD_FLEXCOMM0_CLKSEL, LPC54_SYSCON_FCLKSEL0);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C1_MASTER
  if (port == 1)
    {
      /* Attach 12 MHz clock to FLEXCOMM1 */

      lpc54_flexcomm1_enableclk();

      /* Set FLEXCOMM1 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM1_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi1_dev;
      priv->base     = LPC54_FLEXCOMM1_BASE;
      priv->irqid    = LPC54_IRQ_FLEXCOMM1;
      priv->fclock   = BOARD_FLEXCOMM1_FCLK;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C1_SCK);
      lpc54_gpio_config(GPIO_I2C1_MOSI);
      lpc54_gpio_config(GPIO_I2C1_MISO);

      /* Set up the FLEXCOMM1 function clock */

      putreg32(BOARD_FLEXCOMM1_CLKSEL, LPC54_SYSCON_FCLKSEL1);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C2_MASTER
  if (port == 2)
    {
      /* Attach 12 MHz clock to FLEXCOMM2 */

      lpc54_flexcomm2_enableclk();

      /* Set FLEXCOMM2 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM2_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi2_dev;
      priv->base     = LPC54_FLEXCOMM2_BASE;
      priv->irqid    = LPC54_IRQ_FLEXCOMM2;
      priv->fclock   = BOARD_FLEXCOMM2_FCLK;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C2_SCK);
      lpc54_gpio_config(GPIO_I2C2_MOSI);
      lpc54_gpio_config(GPIO_I2C2MISO);

      /* Set up the FLEXCOMM2 function clock */

      putreg32(BOARD_FLEXCOMM2_CLKSEL, LPC54_SYSCON_FCLKSEL2);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C3_MASTER
  if (port == 3)
    {
      /* Attach 12 MHz clock to FLEXCOMM3 */

      lpc54_flexcomm3_enableclk();

      /* Set FLEXCOMM3 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM3_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi3_dev;
      priv->base     = LPC54_FLEXCOMM3_BASE;
      priv->irqid    = LPC54_IRQ_FLEXCOMM3;
      priv->fclock   = BOARD_FLEXCOMM3_FCLK;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C3_SCK);
      lpc54_gpio_config(GPIO_I2C3_MOSI);
      lpc54_gpio_config(GPIO_I2C3_MISO);

      /* Set up the FLEXCOMM3 function clock */

      putreg32(BOARD_FLEXCOMM3_CLKSEL, LPC54_SYSCON_FCLKSEL3);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C4_MASTER
  if (port == 4)
    {
      /* Attach 12 MHz clock to FLEXCOMM4 */

      lpc54_flexcomm4_enableclk();

      /* Set FLEXCOMM4 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM4_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi4_dev;
      priv->base     = LPC54_FLEXCOMM4_BASE;
      priv->irqid    = LPC54_IRQ_FLEXCOMM4;
      priv->fclock   = BOARD_FLEXCOMM4_FCLK;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C4_SCK);
      lpc54_gpio_config(GPIO_I2C4_MOSI);
      lpc54_gpio_config(GPIO_I2C4_MISO);

      /* Set up the FLEXCOMM4 function clock */

      putreg32(BOARD_FLEXCOMM4_CLKSEL, LPC54_SYSCON_FCLKSEL4);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C5_MASTER
  if (port == 5)
    {
      /* Attach 12 MHz clock to FLEXCOMM5 */

      lpc54_flexcomm5_enableclk();

      /* Set FLEXCOMM5 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM5_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi5_dev;
      priv->base     = LPC54_FLEXCOMM5_BASE;
      priv->irqid    = LPC54_IRQ_FLEXCOMM5;
      priv->fclock   = BOARD_FLEXCOMM5_FCLK;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C5_SCK);
      lpc54_gpio_config(GPIO_I2C5_MOSI);
      lpc54_gpio_config(GPIO_I2C5_MISO);

      /* Set up the FLEXCOMM5 function clock */

      putreg32(BOARD_FLEXCOMM5_CLKSEL, LPC54_SYSCON_FCLKSEL5);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C6_MASTER
  if (port == 6)
    {
      /* Attach 12 MHz clock to FLEXCOMM6 */

      lpc54_flexcomm6_enableclk();

      /* Set FLEXCOMM6 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM6_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi6_dev;
      priv->base     = LPC54_FLEXCOMM6_BASE;
      priv->irqid    = LPC54_IRQ_FLEXCOMM6;
      priv->fclock   = BOARD_FLEXCOMM6_FCLK;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C6_SCK);
      lpc54_gpio_config(GPIO_I2C6_MOSI);
      lpc54_gpio_config(GPIO_I2C6_MISO);

      /* Set up the FLEXCOMM6 function clock */

      putreg32(BOARD_FLEXCOMM6_CLKSEL, LPC54_SYSCON_FCLKSEL6);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C7_MASTER
  if (port == 7)
    {
      /* Attach 12 MHz clock to FLEXCOMM7 */

      lpc54_flexcomm7_enableclk();

      /* Set FLEXCOMM7 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM7_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi7_dev;
      priv->base     = LPC54_FLEXCOMM7_BASE;
      priv->irqid    = LPC54_IRQ_FLEXCOMM7;
      priv->fclock   = BOARD_FLEXCOMM7_FCLK;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C7_SCK);
      lpc54_gpio_config(GPIO_I2C7_MOSI);
      lpc54_gpio_config(GPIO_I2C7_MISO);

      /* Set up the FLEXCOMM7 function clock */

      putreg32(BOARD_FLEXCOMM7_CLKSEL, LPC54_SYSCON_FCLKSEL7);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C8_MASTER
  if (port == 8)
    {
      /* Attach 12 MHz clock to FLEXCOMM8 */

      lpc54_flexcomm8_enableclk();

      /* Set FLEXCOMM8 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM8_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi8_dev;
      priv->base     = LPC54_FLEXCOMM8_BASE;
      priv->irqid    = LPC54_IRQ_FLEXCOMM8;
      priv->fclock   = BOARD_FLEXCOMM8_FCLK;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C8_SCK);
      lpc54_gpio_config(GPIO_I2C8_MOSI);
      lpc54_gpio_config(GPIO_I2C8_MISO);

      /* Set up the FLEXCOMM8 function clock */

      putreg32(BOARD_FLEXCOMM8_CLKSEL, LPC54_SYSCON_FCLKSEL8);
    }
  else
#endif
#ifdef CONFIG_LPC54_I2C9_MASTER
  if (port == 9)
    {
      /* Attach 12 MHz clock to FLEXCOMM9 */

      lpc54_flexcomm9_enableclk();

      /* Set FLEXCOMM9 to the SPI peripheral, locking that configuration
       * in place.
       */

      putreg32(FLEXCOMM_PSELID_PERSEL_SPI | FLEXCOMM_PSELID_LOCK,
               LPC54_FLEXCOMM9_PSELID);

      /* Initialize the state structure */

      priv           = &g_spi9_dev;
      priv->base     = LPC54_FLEXCOMM9_BASE;
      priv->irqid    = LPC54_IRQ_FLEXCOMM9;
      priv->fclock   = BOARD_FLEXCOMM9_FCLK;

      /* Configure SPI pins (defined in board.h) */

      lpc54_gpio_config(GPIO_I2C9_SCK);
      lpc54_gpio_config(GPIO_I2C9_MOSI);
      lpc54_gpio_config(GPIO_I2C9_MISO);

      /* Set up the FLEXCOMM9 function clock */

      putreg32(BOARD_FLEXCOMM9_CLKSEL, LPC54_SYSCON_FCLKSEL9);
    }
  else
#endif
    {
      return NULL;
    }

  leave_critical_section(flags);

  /* Enable the SPI peripheral*/
  /* Configure 8-bit SPI mode and master mode */
#warning Missing logic

  /* Set the initial SPI configuration */

  priv->frequency = 0;
  priv->nbits     = 8;
  priv->mode      = SPIDEV_MODE0;
  priv->dev.ops   = &g_spi_ops;

  /* Select a default frequency of approx. 400KHz */

  lpc54_spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);
  return &priv->dev;
}

#endif /* HAVE_SPI_MASTER_DEVICE */
