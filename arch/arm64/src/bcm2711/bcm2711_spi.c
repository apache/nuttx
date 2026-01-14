/****************************************************************************
 * arch/arm64/src/bcm2711/bcm2711_spi.c
 *
 * Contributed by Matteo Golin
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include "arm64_arch.h"
#include "arm64_gic.h"
#include "bcm2711_gpio.h"
#include "chip.h"
#include "hardware/bcm2711_aux.h"
#include "hardware/bcm2711_irq.h"
#include "hardware/bcm2711_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Core clock nominal frequency in Hz */

#define CORE_CLOCK_FREQUENCY 150000000

/* Calculate the value required in the CDIV register for the correct
 * frequency.
 */

#define SPI_CDIV(freq) (CORE_CLOCK_FREQUENCY / (freq))

/* Calculate the actual frequency based on the speed field. */

#define SPI_ACTUAL_FREQ(cdiv) (CORE_CLOCK_FREQUENCY / (cdiv))

/* SPI interface pins (GPIO pins) */

/* SPI0 (has alternate pin options, except CE2) */

#ifdef CONFIG_BCM2711_SPI0
#if CONFIG_BCM2711_SPI0_CE0 == 8
#define SPI0_CE0_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_SPI0_CE0 == 36
#define SPI0_CE0_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_SPI0_CE0 == 43
#define SPI0_CE0_ALT BCM_GPIO_FUNC4
#else
#error "Invalid GPIO number for SPI0 CE0"
#endif

#if CONFIG_BCM2711_SPI0_CE1 == 7
#define SPI0_CE1_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_SPI0_CE1 == 35
#define SPI0_CE1_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_SPI0_CE1 == 44
#define SPI0_CE1_ALT BCM_GPIO_FUNC4
#else
#error "Invalid GPIO number for SPI0 CE1"
#endif

#if CONFIG_BCM2711_SPI0_MISO == 9
#define SPI0_MISO_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_SPI0_MISO == 37
#define SPI0_MISO_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_SPI0_MISO == 40
#define SPI0_MISO_ALT BCM_GPIO_FUNC4
#else
#error "Invalid GPIO number for SPI0 MISO"
#endif

#if CONFIG_BCM2711_SPI0_MOSI == 10
#define SPI0_MOSI_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_SPI0_MOSI == 38
#define SPI0_MOSI_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_SPI0_MOSI == 41
#define SPI0_MOSI_ALT BCM_GPIO_FUNC4
#else
#error "Invalid GPIO number for SPI0 MOSI"
#endif

#if CONFIG_BCM2711_SPI0_SCLK == 11
#define SPI0_SCLK_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_SPI0_SCLK == 39
#define SPI0_SCLK_ALT BCM_GPIO_FUNC0
#elif CONFIG_BCM2711_SPI0_SCLK == 42
#define SPI0_SCLK_ALT BCM_GPIO_FUNC4
#else
#error "Invalid GPIO number for SPI0 SCLK"
#endif

#define SPI0_CE2 45
#endif

/* SPI1 */

#define SPI1_MISO 19
#define SPI1_MOSI 20
#define SPI1_SCLK 21
#define SPI1_CE0 18
#define SPI1_CE1 17
#define SPI1_CE2 16

/* SPI2 (datasheet does not mention this interface's available pins) */

/* SPI3 (no CE2) */

#define SPI3_MISO 1
#define SPI3_MOSI 2
#define SPI3_SCLK 3
#define SPI3_CE0 0
#define SPI3_CE1 24

/* SPI4 (no CE2) */

#define SPI4_MISO 5
#define SPI4_MOSI 6
#define SPI4_SCLK 7
#define SPI4_CE0 4
#define SPI4_CE1 25

/* SPI5 (no CE2) */

#define SPI5_MISO 13
#define SPI5_MOSI 14
#define SPI5_SCLK 15
#define SPI5_CE0 12
#define SPI5_CE1 26

/* SPI6 (no CE2) */

#define SPI6_MISO 19
#define SPI6_MOSI 20
#define SPI6_SCLK 21
#define SPI6_CE0 18
#define SPI6_CE1 27

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI driver state for BCM2711. */

struct bcm2711_spidev_s
{
  struct spi_dev_s spidev; /* Externally visible */
  uint32_t base;           /* Base address of SPI interface register */
  mutex_t lock;            /* Mutual exclusion during chip select */
  sem_t wait;              /* Where to wait for transfer completion */
  uint32_t freq;           /* Request clock frequency */
  uint32_t actualfreq;     /* Actual clock frequency */
  uint8_t nbits;           /* Word bit-width */
  uint8_t mode;            /* 0, 1, 2 or 3 */
  uint8_t port;            /* SPI 0-6 */
  bool initialized;        /* Already initialized */
  uint8_t *txbuffer;       /* Transmit buffer for current transfer */
  uint8_t *rxbuffer;       /* Receive buffer for current transfer */
  size_t nwords;           /* Number of words in exchange */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI methods */

static int spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                                 uint32_t frequency);
static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         void *rxbuffer, size_t nwords);
static void spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, const void *buffer,
                         size_t nwords);
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                          size_t nwords);
#endif /* CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Operations for non-auxiliary SPI interfaces */

static const struct spi_ops_s g_spiops =
{
    .lock = spi_lock,
    .setfrequency = spi_setfrequency,
    .setmode = spi_setmode,
    .setbits = spi_setbits,
    .send = spi_send,
    .select = spi_select,
#ifdef CONFIG_SPI_EXCHANGE
    .exchange = spi_exchange,
#else
    .sndblock = spi_sndblock,
    .recvblock = spi_recvblock,
#endif /* CONFIG_SPI_EXCHANGE */
#ifdef CONFIG_SPI_HWFEATURES
    .hwfeatures = NULL,
#endif /* CONFIG_SPI_HWFEATURES */
#ifdef CONFIG_SPI_CALLBACK
    .registercallback = NULL,
    /* TODO when needed */, /* Provided externally */
#else
    .registercallback = NULL, /* Not implemented */
#endif
};

/* True if the interrupt handler for SPI is attached, false otherwise */

static bool g_interrupts = false;

#if defined(CONFIG_BCM2711_SPI0)
static struct bcm2711_spidev_s g_spi0dev =
{
    .spidev =
        {
            .ops = &g_spiops,
        },
    .base = BCM_SPI0_BASEADDR,
    .freq = 0,
    .port = 0,
    .lock = NXMUTEX_INITIALIZER,
    .wait = NXSEM_INITIALIZER(0, 0),
    .txbuffer = NULL,
    .rxbuffer = NULL,
    .nwords = 0,
};
#endif

#if defined(CONFIG_BCM2711_SPI3)
static struct bcm2711_spidev_s g_spi3dev =
{
    .spidev =
        {
            .ops = &g_spiops,
        },
    .base = BCM_SPI3_BASEADDR,
    .freq = 0,
    .port = 3,
    .lock = NXMUTEX_INITIALIZER,
    .wait = NXSEM_INITIALIZER(0, 0),
    .txbuffer = NULL,
    .rxbuffer = NULL,
    .nwords = 0,
};
#endif

#if defined(CONFIG_BCM2711_SPI4)
static struct bcm2711_spidev_s g_spi4dev =
{
    .spidev =
        {
            .ops = &g_spiops,
        },
    .base = BCM_SPI4_BASEADDR,
    .freq = 0,
    .port = 4,
    .lock = NXMUTEX_INITIALIZER,
    .wait = NXSEM_INITIALIZER(0, 0),
    .txbuffer = NULL,
    .rxbuffer = NULL,
    .nwords = 0,
};
#endif

#if defined(CONFIG_BCM2711_SPI5)
static struct bcm2711_spidev_s g_spi5dev =
{
    .spidev =
        {
            .ops = &g_spiops,
        },
    .base = BCM_SPI5_BASEADDR,
    .freq = 0,
    .port = 5,
    .lock = NXMUTEX_INITIALIZER,
    .wait = NXSEM_INITIALIZER(0, 0),
    .txbuffer = NULL,
    .rxbuffer = NULL,
    .nwords = 0,
};
#endif

#if defined(CONFIG_BCM2711_SPI6)
static struct bcm2711_spidev_s g_spi6dev =
{
    .spidev =
        {
            .ops = &g_spiops,
        },
    .base = BCM_SPI6_BASEADDR,
    .freq = 0,
    .port = 6,
    .lock = NXMUTEX_INITIALIZER,
    .wait = NXSEM_INITIALIZER(0, 0),
    .txbuffer = NULL,
    .rxbuffer = NULL,
    .nwords = 0,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_interrupt_en
 *
 * Description:
 *   Enable/disable interrupts for the SPI interfaces.
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   en   - true: enable interrupts, false: disable interrupts
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_interrupt_en(struct bcm2711_spidev_s *priv, bool en)
{
  uint32_t regval;

  DEBUGASSERT(priv);

  if (en)
    {
      regval = (BCM_SPI_CS_INTR | BCM_SPI_CS_INTD);
    }
  else
    {
      regval = 0;
    }

  modreg32(regval, (BCM_SPI_CS_INTR | BCM_SPI_CS_INTD),
           BCM_SPI_CS(priv->base));
}

/****************************************************************************
 * Name: spi_fill_txfifo
 *
 * Description:
 *   Write as much data to the TX FIFO as possible without overflowing it. TX
 *   will also stop if RX is 3/4 full. This function will not do anything if
 *   the SPI device has no data waiting to be written.
 *
 * Input Parameters:
 *   dev - The SPI device to write to.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_fill_txfifo(struct bcm2711_spidev_s *dev)
{
  uint32_t data;
  while ((getreg32(BCM_SPI_CS(dev->base)) & BCM_SPI_CS_TXD) &&
        !(getreg32(BCM_SPI_CS(dev->base)) & BCM_SPI_CS_RXR) &&
        dev->nwords)
    {
      if (dev->txbuffer)
        {
          data = (uint32_t)*dev->txbuffer++;
        }

      putreg32(dev->txbuffer ? data : 0xff, BCM_SPI_FIFO(dev->base));

      dev->nwords--;
    }
}

/****************************************************************************
 * Name: spi_drain_rxfifo
 *
 * Description:
 *   Read as much data from the RX FIFO as possible. Will not do anything if
 *   there is no data to be read.
 *
 * Input Parameters:
 *   dev - The SPI device to write to.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_drain_rxfifo(struct bcm2711_spidev_s *dev)
{
  uint32_t data;

  while (getreg32(BCM_SPI_CS(dev->base)) & BCM_SPI_CS_RXD)
    {
      data = getreg32(BCM_SPI_FIFO(dev->base));
      if (dev->rxbuffer)
        {
          *dev->rxbuffer++ = (uint8_t)data;
        }
    }
}

/****************************************************************************
 * Name: spi_interrupt_handler
 *
 * Description:
 *   The interrupt service routine for a given SPI interface.
 *
 * Input Parameters:
 *   dev - The SPI device to be serviced.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int spi_service_interrupt(struct bcm2711_spidev_s *dev)
{
  int err = 0;

  /* From peripheral data sheet:
   *
   * If DONE is set and data to write (this means it is the first interrupt),
   * write up to 64 bytes to SPI_FIFO. If DONE is set and no more data, set
   * TA = 0. Read trailing data from SPI_FIFO until RXD is 0.
   *
   * If RXR is set read 48 bytes data from SPI_FIFO and if more data to
   * write, write up to 48 bytes to SPI_FIFO
   */

  if (getreg32(BCM_SPI_CS(dev->base)) & BCM_SPI_CS_DONE)
    {
      if (dev->nwords > 0)
        {
          /* While there is space to write and stuff to write, write it to
           * the FIFO. Check here if the RX FIFO needs reading so we don't
           * overflow it either.
           */

          spi_fill_txfifo(dev);
        }
      else
        {
          /* Mark transfer as over, there is nothing left to write */

          modreg32(0, BCM_SPI_CS_TA, BCM_SPI_CS(dev->base));

          /* Drain the remaining data to be read */

          spi_drain_rxfifo(dev);

          /* Clean buffers. */

          dev->txbuffer = NULL;
          dev->rxbuffer = NULL;

          /* Post the semaphore to let the caller know the transfer is over */

          nxsem_post(&dev->wait);
        }
    }

  /* If the RX FIFO needs to be read */

  if (getreg32(BCM_SPI_CS(dev->base)) & BCM_SPI_CS_RXR)
    {
      /* Read RX FIFO into buffer (if provided) until empty */

      spi_drain_rxfifo(dev);

      /* If more information can be written now, then write it */

      spi_fill_txfifo(dev);
    }

  return err;
}

/****************************************************************************
 * Name: spi_interrupt_handler
 *
 * Description:
 *   The interrupt handler for SPI interrupts.
 *
 * Input Parameters:
 *   irq - The IRQ number
 *   context - The interrupt context
 *   arg - NULL in this case, all data is grabbed from the SPI device global
 *         definitions
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int spi_interrupt_handler(int irq, FAR void *context, FAR void *arg)
{
  uint32_t pactl;

  /* Check which SPI interfaces need to be serviced and service them */

  pactl = getreg32(BCM_PACTL_CS);

  if (pactl & BCM_PACTL_CS_SPI0)
    {
#if defined(CONFIG_BCM2711_SPI0)
      spi_service_interrupt(&g_spi0dev);
#endif
    }
  else if (pactl & BCM_PACTL_CS_SPI1)
    {
#if defined(CONFIG_BCM2711_SPI1)
#error "SPI1 interrupt not implemented"
#endif
    }
  else if (pactl & BCM_PACTL_CS_SPI2)
    {
#if defined(CONFIG_BCM2711_SPI2)
#error "SPI1 interrupt not implemented"
#endif
    }
  else if (pactl & BCM_PACTL_CS_SPI3)
    {
#if defined(CONFIG_BCM2711_SPI3)
      spi_service_interrupt(&g_spi3dev);
#endif
    }
  else if (pactl & BCM_PACTL_CS_SPI4)
    {
#if defined(CONFIG_BCM2711_SPI4)
      spi_service_interrupt(&g_spi4dev);
#endif
    }
  else if (pactl & BCM_PACTL_CS_SPI5)
    {
#if defined(CONFIG_BCM2711_SPI5)
      spi_service_interrupt(&g_spi5dev);
#endif
    }
  else if (pactl & BCM_PACTL_CS_SPI6)
    {
#if defined(CONFIG_BCM2711_SPI6)
      spi_service_interrupt(&g_spi6dev);
#endif
    }

  return 0;
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
 *   Result of locking the mutex, 0 on success.
 *
 ****************************************************************************/

static int spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct bcm2711_spidev_s *priv = (struct bcm2711_spidev_s *)dev;

  if (lock)
    {
      return nxmutex_lock(&priv->lock);
    }
  else
    {
      return nxmutex_unlock(&priv->lock);
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

static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
  struct bcm2711_spidev_s *priv = (struct bcm2711_spidev_s *)dev;

  DEBUGASSERT(priv);

  /* Calculate the clock divisor needed (must be a multiple of 2) */

  uint32_t cdiv = SPI_CDIV(frequency);
  cdiv &= ~0x1; /* Clear last bit to guarantee multiple of 2 rounded down */
  DEBUGASSERT(cdiv < 0xffff);

  /* Save the clock divisor to take effect */

  putreg32(cdiv, BCM_SPI_CLK(priv->base));

  /* Calculate the new actual and save settings */

  priv->freq = frequency;
  priv->actualfreq = SPI_ACTUAL_FREQ(cdiv);

  spiinfo("Frequency %" PRId32 "->%" PRId32 "\n", frequency,
          priv->actualfreq);
  return priv->actualfreq;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI mode requested
 *
 * Returned Value: None
 *
 ****************************************************************************/

static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct bcm2711_spidev_s *priv = (struct bcm2711_spidev_s *)dev;
  uint32_t regval;

  DEBUGASSERT(priv);

  /* No change to SPI mode */

  if (mode == priv->mode)
    {
      return;
    }

  switch (mode)
    {
    case SPIDEV_MODE0: /* CPOL=0, CPHA=0 */
      regval = 0;
      break;
    case SPIDEV_MODE1: /* CPOL=0, CPHA=1 */
      regval = BCM_SPI_CS_CPHA;
      break;
    case SPIDEV_MODE2: /* CPOL=1, CPHA=0 */
      regval = BCM_SPI_CS_CPOL;
      break;
    case SPIDEV_MODE3: /* CPOL=1, CPHA=1 */
      regval = BCM_SPI_CS_CPOL | BCM_SPI_CS_CPHA;
      break;
    default:
      spierr("Bad SPI mode: %d\n", mode);
      DEBUGASSERT(false);
      return;
    }

  spiinfo("SPI set mode: %d", mode);
  modreg32(regval, (BCM_SPI_CS_CPOL | BCM_SPI_CS_CPHA),
           BCM_SPI_CS(priv->base));
  priv->mode = mode;
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
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
  struct bcm2711_spidev_s *priv = (struct bcm2711_spidev_s *)dev;

  DEBUGASSERT(priv);

  /* As far as I'm aware, the BCM2711 only supports 8 bits per word? */

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
 *   wd  - The word to send. The size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct bcm2711_spidev_s *priv = (struct bcm2711_spidev_s *)dev;
  uint32_t regval;

  /* Wait until the TX FIFO can accept data */

  while (!(getreg32(BCM_SPI_CS(priv->base)) & BCM_SPI_CS_TXD))
    ;

  /* Make sure TA (transfer active) bit is set */

  modreg32(BCM_SPI_CS_TA, BCM_SPI_CS_TA, BCM_SPI_CS(priv->base));

  /* Write the byte to the TX FIFO */

  putreg32(wd, BCM_SPI_FIFO(priv->base));

  /* Wait for the RX FIFO not empty */

  while (!(getreg32(BCM_SPI_CS(priv->base)) & BCM_SPI_CS_RXD))
    ;

  /* Get the value from the RX FIFO */

  regval = getreg32(BCM_SPI_FIFO(priv->base));
  spiinfo("%04" PRIx32 "->%04" PRIx32 "\n", wd, regval);

  /* Wait for done signal */

  while (!(getreg32(BCM_SPI_CS(priv->base)) & BCM_SPI_CS_DONE))
    ;

  /* End the transfer */

  modreg32(0, BCM_SPI_CS_TA, BCM_SPI_CS(priv->base));

  return regval;
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Wrapper function to exchange a block of data from SPI.
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

static void spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                         void *rxbuffer, size_t nwords)
{
  struct bcm2711_spidev_s *priv = (struct bcm2711_spidev_s *)dev;

  /* Set the buffers up */

  priv->txbuffer = (void *)txbuffer;
  priv->rxbuffer = rxbuffer;
  priv->nwords = nwords;

  /* Start the transfer */

  modreg32(BCM_SPI_CS_TA, BCM_SPI_CS_TA, BCM_SPI_CS(priv->base));

  /* Block on semaphore until transfers are all done */

  nxsem_wait_uninterruptible(&priv->wait);
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

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                         size_t nwords)
{
  return spi_exchange(dev, buffer, NULL, nwords);
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
 *bits-per-word selected for the SPI interface.  If nbits <= 8, the data is
 *            packed into uint8_t's; if nbits >8, the data is packed into
 *            uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_recvblock(struct spi_dev_s *dev, void *buffer, size_t nwords)
{
  return spi_exchange(dev, NULL, buffer, nwords);
}
#endif /* !CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Name: spi_select
 *
 * Description:
 *   Enables/disables the chosen chip select ping for the BCM2711 SPI
 *   interface. Some SPI interfaces have up to 3 chip select options. The CS
 *   pin is automatically asserted and de-asserted during transfers, so this
 *   function will only choose which CS interface is active at any time.
 *
 ****************************************************************************/

static void spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  struct bcm2711_spidev_s *priv = (struct bcm2711_spidev_s *)dev;
  spiinfo("Select devid %u", devid);

  /* Get the last bits of the device ID, indicating index (TODO: is this
   * appropriate?)
   */

  devid = devid & 0xffff;
  DEBUGASSERT(priv);
  DEBUGASSERT(devid < 2);
  spiinfo("Using chip enable %u", devid);

  /* Do nothing if not actively selecting a peripheral device */

  if (!selected)
    {
      return;
    }

  /* Set chip select pin to be the active pin */

  modreg32(devid & 0x3, BCM_SPI_CS_CS, BCM_SPI_CS(priv->base));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bcm2711_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI port.
 *
 * Input Parameter:
 *   port - Port number
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure.
 *
 ****************************************************************************/

struct spi_dev_s *bcm2711_spibus_initialize(int port)
{
  int err;
  struct bcm2711_spidev_s *priv = NULL;

  switch (port)
    {
#if defined(CONFIG_BCM2711_SPI0)
    case 0:
      priv = &g_spi0dev;
      break;
#endif
#if defined(CONFIG_BCM2711_SPI1)
    case 1:
#error "SPI1 is not implemented"
      break;
#endif
#if defined(CONFIG_BCM2711_SPI2)
    case 2:
#error "SPI2 is not implemented"
      break;
#endif
#if defined(CONFIG_BCM2711_SPI3)
    case 3:
      priv = &g_spi3dev;
      break;
#endif
#if defined(CONFIG_BCM2711_SPI4)
    case 4:
      priv = &g_spi4dev;
      break;
#endif
#if defined(CONFIG_BCM2711_SPI5)
    case 5:
      priv = &g_spi5dev;
      break;
#endif
#if defined(CONFIG_BCM2711_SPI6)
    case 6:
      priv = &g_spi6dev;
      break;
#endif
    default:
      return NULL;
    }

  if (priv->initialized)
    {
      return &priv->spidev;
    }

  /* Perform initialization */

  priv->nbits = 8;
  priv->freq = 0;
  priv->mode = SPIDEV_MODE0;
  priv->txbuffer = NULL;
  priv->rxbuffer = NULL;
  priv->nwords = 0;

  /* If the main SPI interrupt handler is not attached yet, attach it */

  if (!g_interrupts)
    {
      err = irq_attach(BCM_IRQ_VC_SPI, spi_interrupt_handler, NULL);

      if (err)
        {
          spierr("Could not attach SPI interrupt handler: %d\n", err);
          irqerr("Could not attach SPI interrupt handler: %d\n", err);
          return NULL;
        }

      arm64_gic_irq_set_priority(BCM_IRQ_VC_SPI, 0, IRQ_TYPE_LEVEL);
      up_enable_irq(BCM_IRQ_VC_SPI);
      g_interrupts = true;

      irqinfo("SPI interrupt handler attached.");
      spiinfo("SPI interrupt handler attached.");
    }

  /* Enable SPI interface interrupts for non-auxiliary SPI devices */

  if (port != 1 && port != 2)
    {
      spiinfo("Enabled interrupts for SPI%d\n", port);
      spi_interrupt_en(priv, true);
    }

  /* Set up GPIO pins for the interface */

  switch (port)
    {
#if defined(CONFIG_BCM2711_SPI0)
    case 0:
      bcm2711_gpio_set_pulls(CONFIG_BCM2711_SPI0_CE0, true, false);
      bcm2711_gpio_set_pulls(CONFIG_BCM2711_SPI0_CE1, true, false);
      bcm2711_gpio_set_pulls(CONFIG_BCM2711_SPI0_MOSI, false, true);
      bcm2711_gpio_set_pulls(CONFIG_BCM2711_SPI0_MISO, false, true);
      bcm2711_gpio_set_pulls(CONFIG_BCM2711_SPI0_SCLK, false, true);

      bcm2711_gpio_set_func(CONFIG_BCM2711_SPI0_CE0, SPI0_CE0_ALT);
      bcm2711_gpio_set_func(CONFIG_BCM2711_SPI0_CE1, SPI0_CE1_ALT);
      bcm2711_gpio_set_func(CONFIG_BCM2711_SPI0_MOSI, SPI0_MOSI_ALT);
      bcm2711_gpio_set_func(CONFIG_BCM2711_SPI0_MISO, SPI0_MISO_ALT);
      bcm2711_gpio_set_func(CONFIG_BCM2711_SPI0_SCLK, SPI0_SCLK_ALT);
      break;
#endif
#if defined(CONFIG_BCM2711_SPI1)
    case 1:
      bcm2711_gpio_set_pulls(SPI1_CE0, true, false);
      bcm2711_gpio_set_pulls(SPI1_CE1, true, false);
      bcm2711_gpio_set_pulls(SPI2_CE1, true, false);
      bcm2711_gpio_set_pulls(SPI1_MOSI, false, true);
      bcm2711_gpio_set_pulls(SPI1_MISO, false, true);
      bcm2711_gpio_set_pulls(SPI1_SCLK, false, true);

      bcm2711_gpio_set_func(SPI1_CE0, BCM_GPIO_FUNC4);
      bcm2711_gpio_set_func(SPI1_CE1, BCM_GPIO_FUNC4);
      bcm2711_gpio_set_func(SPI1_CE2, BCM_GPIO_FUNC4);
      bcm2711_gpio_set_func(SPI1_MOSI, BCM_GPIO_FUNC4);
      bcm2711_gpio_set_func(SPI1_MISO, BCM_GPIO_FUNC4);
      bcm2711_gpio_set_func(SPI1_SCLK, BCM_GPIO_FUNC4);
      break;
#endif
#if defined(CONFIG_BCM2711_SPI2)
    case 2:
#error "SPI2 is not implemented"
      break;
#endif
#if defined(CONFIG_BCM2711_SPI3)
    case 3:
      bcm2711_gpio_set_pulls(SPI3_CE0, true, false);
      bcm2711_gpio_set_pulls(SPI3_CE1, true, false);
      bcm2711_gpio_set_pulls(SPI3_MOSI, false, true);
      bcm2711_gpio_set_pulls(SPI3_MISO, false, true);
      bcm2711_gpio_set_pulls(SPI3_SCLK, false, true);

      bcm2711_gpio_set_func(SPI3_CE0, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(SPI3_CE1, BCM_GPIO_FUNC5);
      bcm2711_gpio_set_func(SPI3_MOSI, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(SPI3_MISO, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(SPI3_SCLK, BCM_GPIO_FUNC3);
      break;
#endif
#if defined(CONFIG_BCM2711_SPI4)
    case 4:
      bcm2711_gpio_set_pulls(SPI4_CE0, true, false);
      bcm2711_gpio_set_pulls(SPI4_CE1, true, false);
      bcm2711_gpio_set_pulls(SPI4_MOSI, false, true);
      bcm2711_gpio_set_pulls(SPI4_MISO, false, true);
      bcm2711_gpio_set_pulls(SPI4_SCLK, false, true);

      bcm2711_gpio_set_func(SPI4_CE0, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(SPI4_CE1, BCM_GPIO_FUNC5);
      bcm2711_gpio_set_func(SPI4_MOSI, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(SPI4_MISO, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(SPI4_SCLK, BCM_GPIO_FUNC3);
      break;
#endif
#if defined(CONFIG_BCM2711_SPI5)
    case 5:
      bcm2711_gpio_set_pulls(SPI5_CE0, true, false);
      bcm2711_gpio_set_pulls(SPI5_CE1, true, false);
      bcm2711_gpio_set_pulls(SPI5_MOSI, false, true);
      bcm2711_gpio_set_pulls(SPI5_MISO, false, true);
      bcm2711_gpio_set_pulls(SPI5_SCLK, false, true);

      bcm2711_gpio_set_func(SPI5_CE0, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(SPI5_CE1, BCM_GPIO_FUNC5);
      bcm2711_gpio_set_func(SPI5_MOSI, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(SPI5_MISO, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(SPI5_SCLK, BCM_GPIO_FUNC3);
      break;
#endif
#if defined(CONFIG_BCM2711_SPI6)
    case 6:
      bcm2711_gpio_set_pulls(SPI6_CE0, true, false);
      bcm2711_gpio_set_pulls(SPI6_CE1, true, false);
      bcm2711_gpio_set_pulls(SPI6_MOSI, false, true);
      bcm2711_gpio_set_pulls(SPI6_MISO, false, true);
      bcm2711_gpio_set_pulls(SPI6_SCLK, false, true);

      bcm2711_gpio_set_func(SPI6_CE0, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(SPI6_CE1, BCM_GPIO_FUNC5);
      bcm2711_gpio_set_func(SPI6_MOSI, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(SPI6_MISO, BCM_GPIO_FUNC3);
      bcm2711_gpio_set_func(SPI6_SCLK, BCM_GPIO_FUNC3);
      break;
#endif
    }

  priv->initialized = true;
  return &priv->spidev;
}
