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
#include <errno.h>
#include <stdint.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include "arm64_arch.h"
#include "arm64_gic.h"
#include "chip.h"
#include "hardware/bcm2711_aux.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Core clock nominal frequency in Hz */

#define CORE_CLOCK_FREQUENCY 150000000

/* Calculate the value required in the speed register for the correct
 * frequency.
 */

#define SPI_SPEED_FIELD(spiclk) ((CORE_CLOCK_FREQUENCY / (2 * (spiclk))) - 1)

/* Calculate the actual frequency based on the speed field. */

#define SPI_ACTUAL_FREQ(speedfield)                                            \
  (CORE_CLOCK_FREQUENCY / (2 * ((speedfield) + 1)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI driver state for BCM2711. */

struct bcm2711_spidev_s
{
  struct spi_dev_s spidev; /* Externally visible */
  uint32_t base;           /* Base address of SPI interface register */
  mutex_t lock;            /* Mutual exclusion during chip select */
  uint32_t freq;           /* Request clock frequency */
  uint32_t actualfreq;     /* Actual clock frequency */
  uint8_t nbits;           /* Word bit-width */
  uint8_t mode;            /* 0, 1, 2 or 3 */
  uint8_t port;            /* 1 or 2 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI methods */

static int spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency);
static void spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void spi_setbits(struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(struct spi_dev_s *dev, uint32_t wd);
static void unused_code spi_exchange(struct spi_dev_s *dev,
                                     const void *txbuffer, void *rxbuffer,
                                     size_t nwords);

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(struct spi_dev_s *dev, const void *buffer,
                         size_t nwords);
static void spi_recvblock(struct spi_dev_s *dev, void *buffer,
                          size_t nwords);
#endif /* CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Operations for SPI interfaces */

static const struct spi_ops_s g_spiops =
{
  .lock = spi_lock,
  .setfrequency = spi_setfrequency,
  .setmode = spi_setmode,
  .setbits = spi_setbits,
  .send = spi_send,
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
  .registercallback = /* TODO */, /* Provided externally */
#else
  .registercallback = 0, /* Not implemented */
#endif
};

#define CONFIG_BCM2711_SPI1 // TODO remove
#if defined(CONFIG_BCM2711_SPI1)

static struct bcm2711_spidev_s g_spi1dev =
{
  .spidev =
      {
          .ops = &g_spiops,
      },
  .base = BCM_AUX_SPI1_BASEADDR,
  .frequency = 0,
  .port = 1,
  .lock = NXMUTEX_INITIALIZER,
};

#endif /* defined(CONFIG_BCM2711_SPI1) */

#define CONFIG_BCM2711_SPI2 // TODO remove
#if defined(CONFIG_BCM2711_SPI2)

static struct bcm2711_spidev_s g_spi2dev =
{
  .spidev =
      {
          .ops = &g_spiops,
      },
  .base = BCM_AUX_SPI2_BASEADDR,
  .frequency = 0,
  .port = 2,
  .lock = NXMUTEX_INITIALIZER,
};

#endif /* defined(CONFIG_BCM2711_SPI2) */

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

  /* Calculate the speed field value needed */

  uint32_t speed = SPI_SPEED_FIELD(frequency);

  /* Save the speed field to take effect */

  modreg32(BCM_SPI_CNTL0_SPEED, speed << 20, BCM_SPI_CNTL0_REG(priv->base));

  /* Calculate the new actual and save settings */

  priv->freq = frequency;
  priv->actualfreq = SPI_ACTUAL_FREQ(speed);

  spiinfo("Frequency %" PRId32 "->%" PRId32 "\n",
          frequency, priv->actualfreq);
  return priv->actualfreq;
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
  return NULL;
}
