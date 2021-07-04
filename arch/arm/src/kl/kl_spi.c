/****************************************************************************
 * arch/arm/src/kl/kl_spi.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "kl_spi.h"
#include "kl_gpio.h"
#include "hardware/kl_memorymap.h"
#include "hardware/kl_sim.h"
#include "hardware/kl_spi.h"
#include "hardware/kl_pinmux.h"

#if defined(CONFIG_KL_SPI0) || defined(CONFIG_KL_SPI1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kl_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         spibase;    /* Base address of SPI registers */
  sem_t            exclsem;    /* Held while chip is selected for mutual exclusion */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          nbits;      /* Width of word in bits (8 to 16) */
  uint8_t          mode;       /* Mode 0,1,2,3 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint8_t spi_getreg(FAR struct kl_spidev_s *priv,
                                 uint8_t offset);
static inline void spi_putreg(FAR struct kl_spidev_s *priv, uint8_t offset,
                              uint8_t value);

/* SPI methods */

static int      spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                                 uint32_t frequency);
static void     spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void     spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
static void     spi_exchange(FAR struct spi_dev_s *dev,
                             FAR const void *txbuffer, FAR void *rxbuffer,
                             size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void     spi_sndblock(FAR struct spi_dev_s *dev,
                             FAR const void *txbuffer, size_t nwords);
static void     spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                              size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_KL_SPI0
static const struct spi_ops_s g_spi0ops =
{
  .lock              = spi_lock,
  .select            = kl_spi0select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = 0,           /* Not supported */
#endif
  .status            = kl_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = kl_spi0cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,           /* Not supported */
};

static struct kl_spidev_s g_spi0dev =
{
  .spidev            =
    {
      &g_spi0ops
    },
  .spibase           = KL_SPI0_BASE,
};
#endif

#ifdef CONFIG_KL_SPI1
static const struct spi_ops_s g_spi1ops =
{
  .lock              = spi_lock,
  .select            = kl_spi1select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = kl_spi1status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = kl_spi1cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
  .registercallback  = 0,
};

static struct kl_spidev_s g_spi1dev =
{
  .spidev            =
    {
      &g_spi1ops
    },
  .spibase           = KL_SPI1_BASE,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

static inline uint8_t spi_getreg(FAR struct kl_spidev_s *priv,
                                 uint8_t offset)
{
  return getreg8(priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 16-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 16-bit value to be written
 *
 * Returned Value:
 *   The contents of the 16-bit register
 *
 ****************************************************************************/

static inline void spi_putreg(FAR struct kl_spidev_s *priv, uint8_t offset,
                              uint8_t value)
{
  putreg8(value, priv->spibase + offset);
}

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

static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct kl_spidev_s *priv = (FAR struct kl_spidev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&priv->exclsem);
    }
  else
    {
      ret = nxsem_post(&priv->exclsem);
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

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  FAR struct kl_spidev_s *priv = (FAR struct kl_spidev_s *)dev;
  uint32_t divisor;
  uint32_t actual;
  unsigned int spr;
  unsigned int sppr;

  /* Check if the requested frequence is the same as the frequency
   * selection.
   */

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency.  Return the actual. */

      return priv->actual;
    }

  /* The clock source for the SPI baud rate generator is the bus clock.   We
   * need to pick  a prescaler value 1, 2, 3, 4, 5, 6, 7, or 8 and then a
   * divisor in the range {2, 4, 8, 16, 32, 64, 128, 256, or 512).
   *
   *
   * BaudRateDivisor = (SPPR + 1) × 2^(SPR + 1)
   * BaudRate = BusClock / BaudRateDivisor
   *
   * The strategy is to pick the smallest divisor that yields an in-range
   * solution.  I am not sure if this *always* results in an optimal
   * solution.  But consider, for example, with a 24Mhz bus clock and a
   * target of 400KHz
   *
   *   target divisor is 24,000,000 / 400,000 = 60
   *   spr = 1 -> sppr = 60 / (1 << 1) = 30 -> out of range
   *   spr = 2 -> sppr = 60 / (1 << 2) = 15 -> out of range
   *   spr = 3 -> sppr = 60 / (1 << 3) = 7  -> actual = 24000000 / 7 * 8
   *       = 428571
   *   spr = 4 -> sppr = 60 / (1 << 4) = 3  -> actual = 24000000 / 3 * 16
   *       = 500000
   *   spr = 5 -> sppr = 60 / (1 << 5) = 1  -> actual = 24000000 / 1 * 32
   *       = 750000
   */

  divisor = BOARD_BUSCLK_FREQ / frequency;
  for (spr = 1; spr < 10; spr++)
    {
      sppr = divisor / (1 << spr);
      if (sppr < 9)
        {
          break;
        }
    }

  /* Handle failures to find a solution by forcing spr to the maximum value */

  DEBUGASSERT(spr < 10);
  if (spr > 9)
    {
      spr  = 9;
      sppr = divisor / 512;
    }

  /* Write the new dividers to the BR register */

  spi_putreg(priv, KL_SPI_BR_OFFSET, SPI_BR_SPR_DIV(spr) |
             SPI_BR_SPPR(sppr));

  /* Calculate the actual divisor and frequency */

  divisor = sppr * (1 << spr);
  actual  = BOARD_BUSCLK_FREQ / divisor;

  /* Save the frequency setting */

  priv->frequency = frequency;
  priv->actual    = actual;

  spiinfo("Frequency %" PRId32 "->%" PRId32 "\n", frequency, actual);
  return actual;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct kl_spidev_s *priv = (FAR struct kl_spidev_s *)dev;
  uint8_t regval;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Yes... Set C1 appropriately */

      regval = spi_getreg(priv, KL_SPI_C1_OFFSET);
      regval &= ~(SPI_C1_CPOL | SPI_C1_CPHA);

      switch (mode)
        {
        case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
          break;

        case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
          regval |= SPI_C1_CPHA;
          break;

        case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
          regval |= SPI_C1_CPOL;
          break;

        case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
          regval |= (SPI_C1_CPOL | SPI_C1_CPHA);
          break;

        default:
          DEBUGASSERT(FALSE);
          return;
        }

      spi_putreg(priv, KL_SPI_C1_OFFSET, regval);

      /* Save the mode so that subsequent re-configurations will be faster */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  /* Only 8-bit mode is supported */

  DEBUGASSERT(nbits == 8);
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

static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct kl_spidev_s *priv = (FAR struct kl_spidev_s *)dev;

  /* Make sure that the transmit buffer is empty */

  while ((spi_getreg(priv, KL_SPI_S_OFFSET) & SPI_S_SPTEF) == 0);

  /* Write the data to transmitted to the SPI Data Register */

  spi_putreg(priv, KL_SPI_D_OFFSET, (uint8_t)wd);

  /* Wait for the SPRF bit in the SPI Status Register to be set to 1. SPRF
   * is set at the completion of an SPI transfer to indicate that received
   * data may be read from the SPI data register
   */

  while ((spi_getreg(priv, KL_SPI_S_OFFSET) & SPI_S_SPRF) == 0);

  /* Return the data */

  return (uint32_t)spi_getreg(priv, KL_SPI_D_OFFSET);
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  FAR struct kl_spidev_s *priv = (FAR struct kl_spidev_s *)dev;
  FAR uint8_t *rxptr = (FAR uint8_t *)rxbuffer;
  FAR uint8_t *txptr = (FAR uint8_t *)txbuffer;
  uint8_t data;

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Loop, sending each word in the user-provied data buffer. */

  for (; nwords > 0; nwords--)
    {
      /* Get the data to send (0xff if there is no data source) */

      if (txptr)
        {
          data = (uint8_t)*txptr++;
        }
      else
        {
          data = 0xff;
        }

      /* Wait for any previous data written to the TDR to be transferred
       * to the serializer.
       */

      while ((spi_getreg(priv, KL_SPI_S_OFFSET) & SPI_S_SPTEF) == 0);

      /* Write the data to transmitted to the Transmit Data Register (TDR) */

      spi_putreg(priv, KL_SPI_D_OFFSET, data);

      /* Wait for the read data to be available in the data register */

      while ((spi_getreg(priv, KL_SPI_S_OFFSET) & SPI_S_SPRF) == 0);

      /* Read the received data from the SPI Data Register..
       * TODO: The following only works if nbits <= 8.
       */

      data = spi_getreg(priv, KL_SPI_D_OFFSET);
      if (rxptr)
        {
          *rxptr++ = (uint8_t)data;
        }
    }
}

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of
 *              words.  The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.  If nbits <= 8,
 *              the data is packed into uint8_t's; if nbits >8, the data is
 *              packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return spi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in
 *              number of words.  The wordsize is determined by the number of
 *              bits-per-word selected for the SPI interface.  If nbits <= 8,
 *              the data is packed into uint8_t's; if nbits >8, the data is
 *              packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                          size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kl_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *kl_spibus_initialize(int port)
{
  FAR struct kl_spidev_s *priv;
  uint32_t regval;

  /* Configure multiplexed pins as connected on the board.  Chip select pins
   * must be configured by board-specific logic.  Most SPI pins multiple,
   * alternative pin selection.  Definitions in the board.h file must be\
   * provided to resolve the board-specific pin configuration like:
   *
   * #define PIN_SPI0_SCK PIN_SPI0_SCK_1
   */

#ifdef CONFIG_KL_SPI0
  if (port == 0)
    {
      priv = &g_spi0dev;

      /* Configure pins for SPI0 */

      kl_configgpio(PIN_SPI0_SCK);
      kl_configgpio(PIN_SPI0_MISO);
      kl_configgpio(PIN_SPI0_MOSI);

      /* Enable clocking */

      regval  = getreg32(KL_SIM_SCGC4);
      regval |= SIM_SCGC4_SPI0;
      putreg32(regval, KL_SIM_SCGC4);
    }
  else
#endif
#ifdef CONFIG_KL_SPI1
  if (port == 1)
    {
      priv = &g_spi1dev;

      /* Configure pins for SPI1 */

      kl_configgpio(PIN_SPI1_SCK);
      kl_configgpio(PIN_SPI1_MISO);
      kl_configgpio(PIN_SPI1_MOSI);

      /* Enable clocking */

      regval  = getreg32(KL_SIM_SCGC4);
      regval |= SIM_SCGC4_SPI1;
      putreg32(regval, KL_SIM_SCGC4);
    }
  else
#endif
    {
      spierr("ERROR: Port %d not configured\n", port);
      return NULL;
    }

  /* Configure master mode, select mode 0, MSB first.  Disable interrupts. */

  spi_putreg(priv, KL_SPI_C1_OFFSET, SPI_C1_SPE | SPI_C1_MSTR);

  /* Disable interrupts, DMA, bidirectional mode, stop-in-wait mode, enable
   * master mode fault detection
   */

  spi_putreg(priv, KL_SPI_C2_OFFSET, 0);

  /* Set the initial SPI configuration */

  priv->frequency = 0;
  priv->mode      = SPIDEV_MODE0;

  /* Select a default frequency of approx. 400KHz */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);
  return &priv->spidev;
}

#endif /* CONFIG_KL_SPI0 || CONFIG_KL_SPI1 */
